import UIKit
import AVFoundation
import VideoToolbox

final class RemoteVideoView: UIView {
    override class var layerClass: AnyClass { AVSampleBufferDisplayLayer.self }
    var displayLayer: AVSampleBufferDisplayLayer { layer as! AVSampleBufferDisplayLayer }
}

@MainActor
final class RemoteVideoCodec {
    struct Frame: Codable {
        let keyframe: Bool
        let parameters: [Data]
        let payload: Data
    }

    var onEncoded: ((Data, Bool) -> Void)?
    var onFrame: ((CGSize) -> Void)?
    weak var view: RemoteVideoView?
    private var encoder: VTCompressionSession?
    private var pool: CVPixelBufferPool?
    private var dimensions = CGSize.zero
    private var busy = false
    private var generation = UUID()
    private var forceKeyframe = true
    private var format: CMVideoFormatDescription?
    private var parameters: [Data] = []
    private var waitingForKeyframe = true
    private var bitrate = 2_000_000

    func reset() {
        generation = UUID()
        if let encoder { VTCompressionSessionInvalidate(encoder) }
        encoder = nil
        pool = nil
        dimensions = .zero
        busy = false
        forceKeyframe = true
        format = nil
        parameters = []
        waitingForKeyframe = true
        view?.displayLayer.sampleBufferRenderer.flush(removingDisplayedImage: true)
    }

    func requestKeyframe() {
        forceKeyframe = true
        bitrate = max(350_000, bitrate * 3 / 4)
        if let encoder {
            VTSessionSetProperty(encoder, key: kVTCompressionPropertyKey_AverageBitRate, value: NSNumber(value: bitrate))
        }
    }

    func encode(_ image: UIImage) {
        guard !busy, let cgImage = image.cgImage else { return }
        let width = max(2, cgImage.width / 2 * 2)
        let height = max(2, cgImage.height / 2 * 2)
        if dimensions != CGSize(width: width, height: height) {
            reset()
            dimensions = CGSize(width: width, height: height)
            let attributes: [String: Any] = [
                kCVPixelBufferPixelFormatTypeKey as String: kCVPixelFormatType_32BGRA,
                kCVPixelBufferWidthKey as String: width,
                kCVPixelBufferHeightKey as String: height,
                kCVPixelBufferIOSurfacePropertiesKey as String: [:]
            ]
            guard CVPixelBufferPoolCreate(nil, nil, attributes as CFDictionary, &pool) == kCVReturnSuccess else { return }
            guard VTCompressionSessionCreate(allocator: nil, width: Int32(width), height: Int32(height),
                codecType: kCMVideoCodecType_H264, encoderSpecification: nil, imageBufferAttributes: nil,
                compressedDataAllocator: nil, outputCallback: nil, refcon: nil,
                compressionSessionOut: &encoder) == noErr, let encoder else { reset(); return }
            VTSessionSetProperty(encoder, key: kVTCompressionPropertyKey_RealTime, value: kCFBooleanTrue)
            VTSessionSetProperty(encoder, key: kVTCompressionPropertyKey_AllowFrameReordering, value: kCFBooleanFalse)
            VTSessionSetProperty(encoder, key: kVTCompressionPropertyKey_ProfileLevel, value: kVTProfileLevel_H264_Main_AutoLevel)
            VTSessionSetProperty(encoder, key: kVTCompressionPropertyKey_AverageBitRate, value: NSNumber(value: bitrate))
            VTSessionSetProperty(encoder, key: kVTCompressionPropertyKey_MaxKeyFrameInterval, value: NSNumber(value: 20))
            VTSessionSetProperty(encoder, key: kVTCompressionPropertyKey_MaxKeyFrameIntervalDuration, value: NSNumber(value: 2))
            VTSessionSetProperty(encoder, key: kVTCompressionPropertyKey_ExpectedFrameRate, value: NSNumber(value: 10))
            VTCompressionSessionPrepareToEncodeFrames(encoder)
        }
        guard let pool, let encoder else { return }
        var buffer: CVPixelBuffer?
        guard CVPixelBufferPoolCreatePixelBuffer(nil, pool, &buffer) == kCVReturnSuccess, let buffer else { return }
        CVPixelBufferLockBaseAddress(buffer, [])
        let context = CGContext(data: CVPixelBufferGetBaseAddress(buffer), width: width, height: height,
                                bitsPerComponent: 8, bytesPerRow: CVPixelBufferGetBytesPerRow(buffer),
                                space: CGColorSpaceCreateDeviceRGB(),
                                bitmapInfo: CGImageAlphaInfo.premultipliedFirst.rawValue | CGBitmapInfo.byteOrder32Little.rawValue)
        context?.draw(cgImage, in: CGRect(x: 0, y: 0, width: width, height: height))
        CVPixelBufferUnlockBaseAddress(buffer, [])
        guard context != nil else { return }
        let properties: CFDictionary? = forceKeyframe ? [kVTEncodeFrameOptionKey_ForceKeyFrame: true] as CFDictionary : nil
        forceKeyframe = false
        busy = true
        let epoch = generation
        let result = VTCompressionSessionEncodeFrame(encoder, imageBuffer: buffer,
            presentationTimeStamp: CMTime(seconds: ProcessInfo.processInfo.systemUptime, preferredTimescale: 1_000_000),
            duration: .invalid, frameProperties: properties, infoFlagsOut: nil) { [weak self] status, _, sample in
                Task { @MainActor in
                    guard let self, self.generation == epoch else { return }
                    self.busy = false
                    guard status == noErr, let sample else { self.forceKeyframe = true; return }
                    self.emit(sample)
                }
            }
        if result != noErr { busy = false; forceKeyframe = true }
    }

    private func emit(_ sample: CMSampleBuffer) {
        guard let format = CMSampleBufferGetFormatDescription(sample), let block = CMSampleBufferGetDataBuffer(sample) else { return }
        let attachments = CMSampleBufferGetSampleAttachmentsArray(sample, createIfNecessary: false) as? [[CFString: Any]]
        let keyframe = attachments?.first?[kCMSampleAttachmentKey_NotSync] as? Bool != true
        var parameters: [Data] = []
        if keyframe {
            var count = 0
            CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, parameterSetIndex: 0,
                parameterSetPointerOut: nil, parameterSetSizeOut: nil, parameterSetCountOut: &count, nalUnitHeaderLengthOut: nil)
            guard count > 0, count <= 8 else { return }
            for index in 0..<count {
                var pointer: UnsafePointer<UInt8>?
                var size = 0
                guard CMVideoFormatDescriptionGetH264ParameterSetAtIndex(format, parameterSetIndex: index,
                    parameterSetPointerOut: &pointer, parameterSetSizeOut: &size,
                    parameterSetCountOut: nil, nalUnitHeaderLengthOut: nil) == noErr,
                      let pointer, size > 0, size <= 65536 else { return }
                parameters.append(Data(bytes: pointer, count: size))
            }
        }
        let length = CMBlockBufferGetDataLength(block)
        guard length > 0, length <= 2 * 1024 * 1024 else { forceKeyframe = true; return }
        var payload = Data(count: length)
        let result = payload.withUnsafeMutableBytes {
            CMBlockBufferCopyDataBytes(block, atOffset: 0, dataLength: length, destination: $0.baseAddress!)
        }
        guard result == noErr,
              let data = try? JSONEncoder().encode(Frame(keyframe: keyframe, parameters: parameters, payload: payload)) else { return }
        onEncoded?(data, keyframe)
    }

    func display(_ data: Data) {
        guard data.count <= 4 * 1024 * 1024,
              let frame = try? JSONDecoder().decode(Frame.self, from: data),
              !frame.payload.isEmpty, frame.payload.count <= 2 * 1024 * 1024,
              frame.parameters.count <= 8,
              frame.parameters.allSatisfy({ !$0.isEmpty && $0.count <= 65536 }),
              let layer = view?.displayLayer else { return }
        if layer.sampleBufferRenderer.status == .failed { layer.sampleBufferRenderer.flush(); waitingForKeyframe = true }
        if waitingForKeyframe && !frame.keyframe { return }
        if frame.keyframe {
            guard frame.parameters.count >= 2 else { return }
            if frame.parameters != parameters {
                let copies = frame.parameters.map { [UInt8]($0) }
                var pointers: [UnsafePointer<UInt8>] = []
                var sizes: [Int] = []
                var created: CMVideoFormatDescription?
                func withParameters(_ index: Int) {
                    if index == copies.count {
                        CMVideoFormatDescriptionCreateFromH264ParameterSets(allocator: nil,
                            parameterSetCount: pointers.count, parameterSetPointers: pointers,
                            parameterSetSizes: sizes, nalUnitHeaderLength: 4, formatDescriptionOut: &created)
                        return
                    }
                    copies[index].withUnsafeBufferPointer { pointer in
                        pointers.append(pointer.baseAddress!)
                        sizes.append(pointer.count)
                        withParameters(index + 1)
                    }
                }
                withParameters(0)
                guard let created else { return }
                let dimensions = CMVideoFormatDescriptionGetDimensions(created)
                guard dimensions.width > 0, dimensions.height > 0,
                      dimensions.width <= 4096, dimensions.height <= 4096 else { return }
                format = created
                parameters = frame.parameters
                layer.sampleBufferRenderer.flush()
            }
            waitingForKeyframe = false
        }
        guard let format else { return }
        if !layer.sampleBufferRenderer.isReadyForMoreMediaData { layer.sampleBufferRenderer.flush(); waitingForKeyframe = true; return }
        var block: CMBlockBuffer?
        guard CMBlockBufferCreateWithMemoryBlock(allocator: kCFAllocatorDefault, memoryBlock: nil,
            blockLength: frame.payload.count, blockAllocator: kCFAllocatorDefault, customBlockSource: nil,
            offsetToData: 0, dataLength: frame.payload.count, flags: 0, blockBufferOut: &block) == noErr,
              let block else { return }
        let copied = frame.payload.withUnsafeBytes {
            CMBlockBufferReplaceDataBytes(with: $0.baseAddress!, blockBuffer: block,
                                          offsetIntoDestination: 0, dataLength: frame.payload.count)
        }
        guard copied == noErr else { return }
        var timing = CMSampleTimingInfo(duration: .invalid, presentationTimeStamp: .zero, decodeTimeStamp: .invalid)
        var size = frame.payload.count
        var sample: CMSampleBuffer?
        guard CMSampleBufferCreateReady(allocator: nil, dataBuffer: block, formatDescription: format,
            sampleCount: 1, sampleTimingEntryCount: 1, sampleTimingArray: &timing,
            sampleSizeEntryCount: 1, sampleSizeArray: &size, sampleBufferOut: &sample) == noErr,
              let sample else { return }
        if let attachments = CMSampleBufferGetSampleAttachmentsArray(sample, createIfNecessary: true) as? [CFMutableDictionary],
           let first = attachments.first {
            CFDictionarySetValue(first, Unmanaged.passUnretained(kCMSampleAttachmentKey_DisplayImmediately).toOpaque(),
                                 Unmanaged.passUnretained(kCFBooleanTrue).toOpaque())
        }
        layer.videoGravity = .resizeAspectFill
        layer.sampleBufferRenderer.enqueue(sample)
        let dimensions = CMVideoFormatDescriptionGetDimensions(format)
        onFrame?(CGSize(width: Int(dimensions.width), height: Int(dimensions.height)))
    }
}
// swift-tools-version: 5.10
import PackageDescription

let package = Package(
    name: "RobotCollisionQueries",
    platforms: [.macOS(.v13), .iOS(.v16)],
    products: [.library(name: "RobotCollisionQueries", targets: ["RobotCollisionQueries"])],
    dependencies: [.package(url: "https://github.com/nicklockwood/Euclid.git", revision: "c9ae7dae32a513a6e8ee1f16fb789416ad8e0f66")],
    targets: [
        .target(name: "CMeshSimplifier", exclude: ["LICENSE.md"], cxxSettings: [.define("MESHOPTIMIZER_NO_WRAPPERS")]),
        .target(name: "RobotCollisionQueries", dependencies: ["Euclid", "CMeshSimplifier"], resources: [.copy("Collision.metal")]),
        .testTarget(name: "RobotCollisionQueriesTests", dependencies: ["RobotCollisionQueries", "Euclid"])
    ],
    cxxLanguageStandard: .cxx11
)
#include <metal_stdlib>
using namespace metal;

struct Triangle { float4 first, second, third; };
struct Node { float4 minimum, maximum; uint4 range; };
struct Job { float4x4 transform; float4 sphere; uint4 source, destination; };

bool allowed(float3 point, float4 sphere) {
    return sphere.w >= 0 && distance(point, sphere.xyz) <= sphere.w;
}

bool intersects(float3 minimum, float3 maximum, Node node) {
    return all(maximum + 1e-6f >= node.minimum.xyz) && all(minimum - 1e-6f <= node.maximum.xyz);
}

bool crosses(float3 start, float3 end, Triangle triangle, float4 sphere) {
    float3 edgeFirst = triangle.second.xyz - triangle.first.xyz;
    float3 edgeSecond = triangle.third.xyz - triangle.first.xyz;
    float3 normal = normalize(cross(edgeFirst, edgeSecond));
    float startSide = dot(normal, start - triangle.first.xyz);
    float endSide = dot(normal, end - triangle.first.xyz);
    if (startSide * endSide >= -1e-14f) return false;
    float3 hit = mix(start, end, startSide / (startSide - endSide));
    float3 relative = hit - triangle.first.xyz;
    float denominator = dot(cross(edgeFirst, edgeSecond), normal);
    float horizontal = dot(cross(relative, edgeSecond), normal) / denominator;
    float vertical = dot(cross(edgeFirst, relative), normal) / denominator;
    return horizontal >= -1e-6f && vertical >= -1e-6f && horizontal + vertical <= 1.000001f && !allowed(hit, sphere);
}

bool surfaceCrosses(Triangle first, Triangle second, float4 sphere) {
    return crosses(first.first.xyz, first.second.xyz, second, sphere)
        || crosses(first.second.xyz, first.third.xyz, second, sphere)
        || crosses(first.third.xyz, first.first.xyz, second, sphere)
        || crosses(second.first.xyz, second.second.xyz, first, sphere)
        || crosses(second.second.xyz, second.third.xyz, first, sphere)
        || crosses(second.third.xyz, second.first.xyz, first, sphere);
}

uint contains(float3 point, uint root, device const Triangle *triangles, device const Node *nodes) {
    Node bounds = nodes[root];
    if (any(point < bounds.minimum.xyz) || any(point > bounds.maximum.xyz)) return 0;
    for (uint attempt = 0; attempt < 3; ++attempt) {
        float3 direction = normalize(attempt == 0 ? float3(1, .371390676f, .694746591f)
            : (attempt == 1 ? float3(.2371f, 1, .5317f) : float3(.7139f, .2931f, 1)));
        uint hits = 0;
        bool ambiguous = false;
        uint cursor = root;
        while (cursor < bounds.range.z) {
            Node node = nodes[cursor];
            float3 nearPlane = (node.minimum.xyz - point) / direction;
            float3 farPlane = (node.maximum.xyz - point) / direction;
            if (max(max(nearPlane.x, nearPlane.y), max(nearPlane.z, 0.f)) > min(min(farPlane.x, farPlane.y), farPlane.z) + 1e-6f) {
                cursor = node.range.z;
                continue;
            }
            for (uint offset = 0; offset < node.range.y; ++offset) {
                Triangle triangle = triangles[node.range.x + offset];
                float3 edgeFirst = triangle.second.xyz - triangle.first.xyz;
                float3 edgeSecond = triangle.third.xyz - triangle.first.xyz;
                float3 perpendicular = cross(direction, edgeSecond);
                float determinant = dot(edgeFirst, perpendicular);
                if (abs(determinant) < 1e-7f * length(edgeFirst) * length(edgeSecond)) continue;
                float3 relative = point - triangle.first.xyz;
                float horizontal = dot(relative, perpendicular) / determinant;
                float3 other = cross(relative, edgeFirst);
                float vertical = dot(direction, other) / determinant;
                float distance = dot(edgeSecond, other) / determinant;
                if (horizontal < -1e-6f || vertical < -1e-6f || horizontal + vertical > 1.000001f || distance < -1e-7f) continue;
                if (abs(distance) <= 1e-7f) return 0;
                if (horizontal <= 1e-6f || vertical <= 1e-6f || horizontal + vertical >= .999999f) ambiguous = true;
                ++hits;
            }
            ++cursor;
        }
        if (!ambiguous) return hits % 2;
    }
    return 2;
}

kernel void collide(device const Triangle *triangles [[buffer(0)]],
                    device const Node *nodes [[buffer(1)]],
                    device const Job *jobs [[buffer(2)]],
                    device atomic_uint *results [[buffer(3)]], uint2 index [[thread_position_in_grid]]) {
    Job job = jobs[index.y];
    if (index.x >= job.source.y || (atomic_load_explicit(&results[job.source.z], memory_order_relaxed) & 1)) return;
    Triangle source = triangles[job.source.x + index.x];
    source.first = job.transform * float4(source.first.xyz, 1);
    source.second = job.transform * float4(source.second.xyz, 1);
    source.third = job.transform * float4(source.third.xyz, 1);
    float3 minimum = min(source.first.xyz, min(source.second.xyz, source.third.xyz));
    float3 maximum = max(source.first.xyz, max(source.second.xyz, source.third.xyz));
    uint root = job.destination.x;
    uint cursor = root;
    while (cursor < nodes[root].range.z) {
        Node node = nodes[cursor];
        if (!intersects(minimum, maximum, node)) { cursor = node.range.z; continue; }
        for (uint offset = 0; offset < node.range.y; ++offset) {
            if (surfaceCrosses(source, triangles[node.range.x + offset], job.sphere)) {
                atomic_fetch_or_explicit(&results[job.source.z], 1, memory_order_relaxed);
                return;
            }
        }
        ++cursor;
    }
    float3 center = (source.first.xyz + source.second.xyz + source.third.xyz) / 3;
    if (job.destination.y && !allowed(center, job.sphere)) {
        uint result = contains(center, root, triangles, nodes);
        if (result) atomic_fetch_or_explicit(&results[job.source.z], result, memory_order_relaxed);
    }
}
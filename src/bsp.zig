const std = @import("std");
const Map = @import("map.zig").Map;
const v3 = @import("geo.zig").v3;
const Plane = @import("geo.zig").Plane;
const Mat3 = @import("math.zig").Mat3;

const pr = std.debug.print;

pub const Bsp = struct {
    normal_id: u32 = 0,
    front: ?*Bsp = null,
    back: ?*Bsp = null,

    pub fn print_digraph(self: *const Bsp) void {
        _ = self;
        std.log.debug("Todo!", .{});
    }
};

const Poly = struct {
    face_id: u32,
};

const GeoData = struct {
    points: []v3,
    normals: []v3,
    faces: [][3]u32,
};

fn planes_from_brushplanes(al: std.mem.Allocator, bps: []Map.Plane) ![]Plane {
    var planes = try std.ArrayListUnmanaged(Plane).initCapacity(al, bps.len);
    for (bps) |bp| {
        try planes.append(al, Plane.from_points(
            bp.a,
            bp.b,
            bp.c,
        ) orelse return error.InvalidPlane);
    }

    return planes.items;
}

fn point_on_plane(pt: v3, plane: Plane) bool {
    const eps = std.math.floatEps(f32);
    return std.math.approxEqAbs(
        f32,
        plane.norm.dot(pt) + plane.offset,
        0.0,
        eps,
    );
}

fn point_in_planes(pt: v3, planes: []const Plane) bool {
    var result = planes.len > 0;
    for (planes) |plane| {
        const eps = std.math.floatEps(f32);
        const pt_offset = plane.norm.dot(pt) + plane.offset;

        const on_plane = std.math.approxEqAbs(f32, pt_offset, 0.0, eps);
        // The plane halfspaces that construct a solid shape define the solid
        // portion as being 'in front of the plane' rather than behind
        const within_volume = pt_offset > 0.0;

        result = result and (within_volume or on_plane);
    }

    return result;
}

fn point_from_planes(a: Plane, b: Plane, c: Plane) ?v3 {
    const c1 = b.norm.cross(c.norm);
    const c2 = c.norm.cross(a.norm);
    const c3 = a.norm.cross(b.norm);

    const numerator =
        c1.mul(-a.offset).add(c2.mul(-b.offset)).add(c3.mul(-c.offset));
    const denominator = a.norm.dot(c1);

    if (std.math.approxEqAbs(f32, denominator, 0.0, std.math.floatEps(f32))) {
        return null;
    }

    return numerator.div(denominator);
}

fn points_to_obj(points: []v3) void {
    pr("o Points\n", .{});
    for (points) |pt| pr("v {d:.1} {d:.1} {d:.1}\n", .{ pt.x, pt.y, pt.z });
}

fn discretize(al: std.mem.Allocator, map: Map) !GeoData {
    // const points = try std.ArrayListUnmanaged(f32).initCapacity(al, 1024);
    // const eps = std.math.floatEps(f32);
    var points = try std.ArrayListUnmanaged(v3).initCapacity(al, 1024);
    const norms: std.AutoHashMapUnmanaged(usize, v3) = .empty;
    _ = norms;

    for (map.entities) |ent| {
        brush: for (ent.brushes, 0..) |brush, brush_i| {
            const planes = try planes_from_brushplanes(al, brush.planes);

            if (planes.len < 3) {
                pr("Invalid brush (too few planes): {}", .{brush_i});
                break :brush;
            }

            for (0..planes.len - 2) |i| {
                for (i + 1..planes.len) |j| {
                    for (j + 1..planes.len) |k| {
                        const a = planes[i];
                        const b = planes[j];
                        const c = planes[k];
                        const pt = point_from_planes(a, b, c) orelse continue;

                        // TODO: Maybe we just need to check if point is in
                        // [a, b, c] only?
                        if (!point_in_planes(pt, planes)) continue;
                        try points.append(al, pt);
                    }
                }
            }

            // pr("Brush: {}\n", .{brush_i});
            // for (planes) |pl| pr("  {}\n", .{pl});
        }
    }

    points_to_obj(points.items);

    // for (map.entities) |ent| {
    //     for (ent.brushes) |brush| {
    //         const planes = try planes_from_brushplanes(al, brush.planes);
    //         for (planes) |a| {
    //             for (planes[1..]) |b| {
    //                 for (planes[2..]) |c| {
    //                     pr("{} {} {}", .{ a, b, c });
    //                 }
    //             }
    //         }
    //     }
    // }

    // outer: for (map.entities) |ent| {
    //     for (ent.brushes) |brush| {
    //         for (brush.planes, 0..) |plane, i| {
    //             const planenorm = norms.get(i) orelse blk: {
    //                 const norm = plane_norm(plane);
    //                 try norms.put(al, i, norm);
    //                 break :blk norm;
    //             };

    //             pr("{} planenorm: {}\n", .{ i, planenorm });

    //             for (i + 1..brush.planes.len) |j| {
    //                 const other_planenorm = norms.get(j) orelse blk: {
    //                     const norm = plane_norm(brush.planes[j]);
    //                     try norms.put(al, j, norm);
    //                     break :blk norm;
    //                 };

    //                 const normcross = planenorm.cross(other_planenorm);
    //                 pr("{} normcross {}: {}\n", .{ i, j, normcross });
    //                 // Skip parallel planes
    //                 if (normcross.mag() < eps) {
    //                     pr("WARN: {}: skipping index {} (parallel)\n", .{
    //                         i,
    //                         j,
    //                     });
    //                     continue;
    //                 }
    //             }
    //         }
    //         break :outer;
    //     }
    // }

    return .{
        .points = &.{},
        .faces = &.{},
        .normals = &.{},
    };
}

pub fn compile(al: std.mem.Allocator, map: Map) !Bsp {
    const data = try discretize(al, map);
    _ = data;

    return .{};
}

test "points in plane" {
    // 45-degree plane that intercepts x-axis at 1, y-axis at 1, and z-axis at 1
    const plane = Plane.from_points(
        v3.make(1.0, 0.0, 0.0),
        v3.make(0.0, 1.0, 0.0),
        v3.make(0.0, 0.0, 1.0),
    ) orelse return error.InvalidPlane;

    const pts = &.{
        v3.make(0.0, 0.0, 0.0),
        v3.make(-2.0, 0.0, 0.0),
        v3.make(0.0, -2.0, 0.0),
        v3.make(0.0, 0.0, -2.0),
    };
    inline for (pts) |pt| {
        try std.testing.expect(point_in_planes(pt, &.{plane}));
    }
}

test "points outside plane" {
    // 45-degree plane that intercepts x-axis at 1, y-axis at 1, and z-axis at 1
    const plane = Plane.from_points(
        v3.make(1.0, 0.0, 0.0),
        v3.make(0.0, 1.0, 0.0),
        v3.make(0.0, 0.0, 1.0),
    ) orelse return error.InvalidPlane;

    const pts = &.{
        v3.make(2.0, 0.0, 0.0),
        v3.make(0.0, 2.0, 0.0),
        v3.make(0.0, 0.0, 2.0),
    };
    inline for (pts) |pt| {
        try std.testing.expect(!point_in_planes(pt, &.{plane}));
    }
}

test "point from planes" {
    const p1 = Plane.from_points(
        v3.make(1.0, 0.0, 0.0),
        v3.make(0.0, 1.0, 0.0),
        v3.make(0.0, 0.0, 1.0),
    ) orelse return error.InvalidPlane;
    const p2 = Plane{ .norm = v3.make(1.0, 0.0, 0.0), .offset = 0.0 };
    const p3 = Plane{ .norm = v3.make(0.0, 0.0, 1.0), .offset = 0.0 };
    const result = point_from_planes(
        p1,
        p2,
        p3,
    ) orelse return error.InvalidPoint;

    const eps = std.math.floatEps(f32);
    try std.testing.expectApproxEqAbs(0.0, result.x, eps);
    try std.testing.expectApproxEqAbs(1.0, result.y, eps);
    try std.testing.expectApproxEqAbs(0.0, result.z, eps);
}

test "point from planes - throws error" {
    const p1 = Plane.from_points(
        v3.make(1.0, 0.0, 0.0),
        v3.make(0.0, 1.0, 0.0),
        v3.make(0.0, 0.0, 1.0),
    ) orelse return error.InvalidPlane;
    const p2 = Plane{ .norm = v3.make(1.0, 0.0, 0.0), .offset = 0.0 };
    const p3 = Plane{ .norm = v3.make(1.0, 0.0, 0.0), .offset = 1.0 };

    try std.testing.expectEqual(null, point_from_planes(p1, p2, p3));
}

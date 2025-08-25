const std = @import("std");
const Map = @import("map.zig").Map;
const v3 = @import("geo.zig").v3;
const Plane = @import("geo.zig").Plane;
const Mat3 = @import("math.zig").Mat3;

const pr = std.debug.print;
// const epsilon: f32 = std.math.floatEps(f32);
const epsilon: f32 = 0.0001;

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
    return std.math.approxEqAbs(
        f32,
        plane.norm.dot(pt) + plane.offset,
        0.0,
        epsilon,
    );
}

fn point_in_planes(pt: v3, planes: []const Plane) bool {
    var result = planes.len > 0;
    for (planes, 0..) |plane, i| {
        _ = i;
        const pt_offset = plane.norm.dot(pt) + plane.offset;

        const on_plane = std.math.approxEqAbs(f32, pt_offset, 0.0, epsilon);
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

fn points_to_obj(name: []const u8, points: []v3, faces: []u32) void {
    pr("o {s}\n", .{name});
    for (points) |pt| pr("v {d:.1} {d:.1} {d:.1}\n", .{ pt.x, pt.y, pt.z });
    var i: u32 = 0;
    while (i < faces.len) : (i += 3) {
        // Obj face indices start at 1
        pr("f {} {} {}\n", .{
            faces[i] + 1,
            faces[i + 1] + 1,
            faces[i + 2] + 1,
        });
    }
}

const PointComparator = struct {
    const Context = struct {
        u: v3,
        v: v3,
        centroid: v3,
        points: []v3,
        print: bool,
    };

    fn less_than(ctx: Context, lhs: u32, rhs: u32) bool {
        const u = ctx.u;
        const v = ctx.v;
        const pts = ctx.points;

        const lhsx = u.dot(pts[lhs]);
        const lhsy = v.dot(pts[lhs]);
        const rhsx = u.dot(pts[rhs]);
        const rhsy = v.dot(pts[rhs]);
        const cx = u.dot(ctx.centroid);
        const cy = v.dot(ctx.centroid);

        const lhs_angle = std.math.atan2(lhsy - cy, lhsx - cx);
        const rhs_angle = std.math.atan2(rhsy - cy, rhsx - cx);

        return lhs_angle < rhs_angle;
    }
};

fn dedupe_points(points: []v3) []v3 {
    if (points.len == 0) return points;

    // Index of the last element
    var end = points.len - 1;

    for (0..end) |i| {
        var j = i + 1;
        while (j < end + 1) : (j += 1) {
            if (points[i].eq(points[j], epsilon)) {
                if (j == end) {
                    end -= 1;
                    continue;
                }

                std.mem.swap(
                    v3,
                    @ptrCast(points.ptr + j),
                    @ptrCast(points.ptr + end),
                );
                end -= 1;
                j -= 1;
            }
        }
    }

    return points[0 .. end + 1];
}

fn discretize(al: std.mem.Allocator, map: Map) !GeoData {
    var pointlist = try std.ArrayListUnmanaged(v3).initCapacity(al, 1024);
    var points: []v3 = undefined;
    var faces = try std.ArrayListUnmanaged(u32).initCapacity(al, 1024);
    var object_name: []const u8 = "Object";

    // Collect points
    // TODO: We're probably going to want to only get the worldspawn brushes at
    // some point instead of getting all the brushes across all entities
    for (map.entities) |ent| {
        for (ent.properties) |prop| {
            pr("{s}: {s}\n", .{ prop.name, prop.value });
            if (std.mem.eql(u8, prop.name, "mapname")) {
                object_name = prop.value;
            }
        }

        brush: for (ent.brushes, 0..) |brush, brush_i| {
            const planes = try planes_from_brushplanes(al, brush.planes);

            if (planes.len < 3) {
                pr("Invalid brush (too few planes): brush #{}", .{brush_i});
                continue :brush;
            }

            for (0..planes.len) |i| {
                j: for (i..planes.len) |j| {
                    if (i == j) continue :j;

                    k: for (j..planes.len) |k| {
                        if (j == k) continue :k;

                        const a = planes[i];
                        const b = planes[j];
                        const c = planes[k];
                        const pt = point_from_planes(a, b, c) orelse continue;

                        if (!point_in_planes(pt, planes)) continue;
                        try pointlist.append(al, pt);
                    }
                }
            }

            points = dedupe_points(pointlist.items);

            // List of u32 indices into the points array
            var plane_pts = try std.ArrayListUnmanaged(u32).initCapacity(
                al,
                points.len,
            );
            for (planes, 0..) |plane, plane_i| {
                // TODO: If this is a bottleneck we should look at using a
                // dictionary to store plane-point relationships somehow
                plane_pts.clearRetainingCapacity();
                for (points, 0..) |pt, i| {
                    if (!point_on_plane(pt, plane)) continue;
                    try plane_pts.append(al, @intCast(i));
                }

                // Triangle faces don't need to be sorted
                if (plane_pts.items.len < 3) continue;
                if (plane_pts.items.len == 3) {
                    try faces.append(al, plane_pts.items[0]);
                    try faces.append(al, plane_pts.items[1]);
                    try faces.append(al, plane_pts.items[2]);
                    continue;
                }

                // var centroid = v3.make(0.0, 0.0, 0.0);
                var centroid = pointlist.items[plane_pts.items[0]];
                for (plane_pts.items[1..]) |pt| centroid = centroid.add(
                    pointlist.items[pt],
                );
                centroid = centroid.div(@floatFromInt(plane_pts.items.len));

                std.sort.pdq(
                    u32,
                    plane_pts.items,
                    PointComparator.Context{
                        .u = brush.planes[plane_i].u,
                        .v = brush.planes[plane_i].v,
                        .centroid = centroid,
                        .points = pointlist.items,
                        .print = plane_i == 7,
                    },
                    PointComparator.less_than,
                );

                // Quads are trivial
                if (plane_pts.items.len == 4) {
                    try faces.append(al, plane_pts.items[0]);
                    try faces.append(al, plane_pts.items[1]);
                    try faces.append(al, plane_pts.items[2]);
                    try faces.append(al, plane_pts.items[0]);
                    try faces.append(al, plane_pts.items[2]);
                    try faces.append(al, plane_pts.items[3]);
                    continue;
                }

                var start: u32 = 0;
                var end: u32 = @intCast(plane_pts.items.len - 1);
                var toggle = true;
                while (end - start >= 2) {
                    var a: u32 = undefined;
                    var b: u32 = undefined;
                    var c: u32 = undefined;
                    if (toggle) {
                        a = start;
                        b = start + 1;
                        c = end;
                        start += 1;
                    } else {
                        a = end;
                        b = start;
                        c = end - 1;
                        end -= 1;
                    }
                    toggle = !toggle;
                    try faces.append(al, plane_pts.items[a]);
                    try faces.append(al, plane_pts.items[b]);
                    try faces.append(al, plane_pts.items[c]);
                }
            }
        }
    }

    // Construct faces
    points_to_obj(object_name, points, faces.items);

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

test "dedupe points" {
    var pt_data = [_]v3{
        v3.make(0.0, 0.0, 0.0),
        v3.make(1.0, 0.0, 0.0),
        v3.make(2.0, 0.0, 0.0),
        v3.make(1.0, 0.0, 0.0),
        v3.make(3.0, 0.0, 0.0),
        v3.make(5.0, 0.0, 0.0),
    };
    var pts: []v3 = pt_data[0..pt_data.len];

    pts = dedupe_points(pts);
    try std.testing.expectEqualSlices(
        v3,
        &[_]v3{
            v3.make(0.0, 0.0, 0.0),
            v3.make(1.0, 0.0, 0.0),
            v3.make(2.0, 0.0, 0.0),
            v3.make(5.0, 0.0, 0.0),
            v3.make(3.0, 0.0, 0.0),
        },
        pts,
    );
}

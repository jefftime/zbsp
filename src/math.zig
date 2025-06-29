const std = @import("std");
const geo = @import("geo.zig");
const v3 = geo.v3;

pub const Mat3 = struct {
    const Self = @This();

    data: [3 * 3]f32,

    // zig fmt: off
    fn init(
        a: f32, b: f32, c: f32,
        d: f32, e: f32, f: f32,
        g: f32, h: f32, i: f32,
    ) Self {
    // zig fmt: on
        return .{
            .data = .{ a, b, c, d, e, f, g, h, i },
        };
    }

    fn from_array(data: [3 * 3]f32) Self {
        return .{ .data = data };
    }
};

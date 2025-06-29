const std = @import("std");
const scanner = @import("scanner.zig");
const geo = @import("geo.zig");
const v3 = geo.v3;

const ParseError = error{ EndOfStream, InvalidToken };

pub const Map = struct {
    pub const Entity = struct {
        properties: []const Property,
        brushes: []const Brush,
    };
    // Right now we're just supporting Valve220 brush format
    pub const Plane = struct {
        a: v3,
        b: v3,
        c: v3,
        texname: []const u8,
        u: v3,
        u_offset: f32,
        v: v3,
        v_offset: f32,
        rot: f32,
        u_scale: f32,
        v_scale: f32,
    };
    pub const Brush = struct { planes: []Plane };
    pub const Property = struct { name: []const u8, value: []const u8 };

    entities: []const Entity,

    pub fn print(self: *const Map) void {
        for (self.entities, 0..) |ent, i| {
            std.log.info("Entity {}:", .{i});
            std.log.info(" Properties:", .{});
            for (ent.properties) |prop| std.log.info("  {s}: {s}", .{
                prop.name,
                prop.value,
            });
            std.log.info(" Brushes:", .{});
            for (ent.brushes, 0..) |b, j| {
                std.log.info("  #{}", .{j});
                for (b.planes) |p| std.log.info(
                    "   ( {d:.1} {d:.1} {d:.1} ) ( {d:.1} {d:.1} {d:.1} ) ( {d:.1} {d:.1} {d:.1} ) {s}",
                    .{
                        p.a.x,
                        p.a.y,
                        p.a.z,
                        p.b.x,
                        p.b.y,
                        p.b.z,
                        p.c.x,
                        p.c.y,
                        p.c.z,
                        p.texname,
                    },
                );
            }
        }

        return;
    }
};

pub fn parse(al: std.mem.Allocator, tokens: []scanner.Token) !Map {
    var tks = scanner.TokenScanner.init(tokens);
    var entities = try std.ArrayListUnmanaged(Map.Entity).initCapacity(al, 128);

    while (tks.peek() != null) try entities.append(al, try entity(al, &tks));

    entities.shrinkAndFree(al, entities.items.len);
    return .{ .entities = entities.items };
}

fn entity(
    al: std.mem.Allocator,
    tks: *scanner.TokenScanner,
) !Map.Entity {
    try tks.expect_sym('{');

    // Properties
    var properties =
        try std.ArrayListUnmanaged(Map.Property).initCapacity(al, 64);
    while (tks.match(.string)) {
        const name = try tks.expect(.string);
        const value = try tks.expect(.string);

        try properties.append(al, Map.Property{
            .name = name.string,
            .value = value.string,
        });
    }
    properties.shrinkAndFree(al, properties.items.len);

    // Brushes
    var brushes = try std.ArrayListUnmanaged(Map.Brush).initCapacity(al, 64);
    entity: while (tks.peek()) |start_token| switch (start_token) {
        .sym => |s| switch (s) {
            '{' => try brushes.append(al, try brush(al, tks)),
            '}' => {
                try tks.expect_sym('}');
                break :entity;
            },
            else => return error.InvalidToken,
        },
        else => return error.InvalidToken,
    };

    return .{ .properties = properties.items, .brushes = brushes.items };
}

fn brush(
    al: std.mem.Allocator,
    tks: *scanner.TokenScanner,
) !Map.Brush {
    var planes = try std.ArrayListUnmanaged(Map.Plane).initCapacity(al, 64);

    try tks.expect_sym('{');
    plane: switch (tks.peek() orelse return error.InvalidToken) {
        .sym => |s| switch (s) {
            '(' => {
                try planes.append(al, try plane(tks));
                continue :plane tks.peek() orelse return error.EndOfStream;
            },
            '}' => {
                try tks.expect_sym('}');
                break :plane;
            },
            else => return error.InvalidToken,
        },
        else => return error.InvalidToken,
    }

    planes.shrinkAndFree(al, planes.items.len);
    return .{ .planes = planes.items };
}

fn float_or_number(tks: *scanner.TokenScanner) !f32 {
    const tk = tks.pop() orelse return error.EndOfStream;
    return switch (tk) {
        .float => |f| f,
        .number => |n| @floatFromInt(n),
        else => error.InvalidToken,
    };
}

fn plane(tks: *scanner.TokenScanner) !Map.Plane {
    var result: Map.Plane = undefined;

    try tks.expect_sym('(');
    result.a = v3{
        .x = try float_or_number(tks),
        .y = try float_or_number(tks),
        .z = try float_or_number(tks),
    };
    try tks.expect_sym(')');
    try tks.expect_sym('(');
    result.b = v3{
        .x = try float_or_number(tks),
        .y = try float_or_number(tks),
        .z = try float_or_number(tks),
    };
    try tks.expect_sym(')');
    try tks.expect_sym('(');
    result.c = v3{
        .x = try float_or_number(tks),
        .y = try float_or_number(tks),
        .z = try float_or_number(tks),
    };
    try tks.expect_sym(')');

    switch (tks.pop() orelse return error.EndOfStream) {
        .ident => |ident| result.texname = ident,
        .string => |str| result.texname = str,
        else => {
            std.log.info("??", .{});
            return error.InvalidToken;
        },
    }

    try tks.expect_sym('[');
    result.u = v3{
        .x = try float_or_number(tks),
        .y = try float_or_number(tks),
        .z = try float_or_number(tks),
    };
    result.u_offset = try float_or_number(tks);
    try tks.expect_sym(']');

    try tks.expect_sym('[');
    result.v = v3{
        .x = try float_or_number(tks),
        .y = try float_or_number(tks),
        .z = try float_or_number(tks),
    };
    result.v_offset = try float_or_number(tks);
    try tks.expect_sym(']');

    result.rot = try float_or_number(tks);
    result.u_scale = try float_or_number(tks);
    result.v_scale = try float_or_number(tks);

    return result;
}

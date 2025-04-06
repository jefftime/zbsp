const std = @import("std");

fn quit(err: ?anyerror) noreturn {
    std.process.exit(if (err) |e| @intCast(@intFromError(e)) else 1);
}

pub fn die(err: ?anyerror, comptime fmt: []const u8, args: anytype) noreturn {
    var bw = std.io.bufferedWriter(std.io.getStdErr().writer());
    const stderr = bw.writer();

    stderr.print(fmt, args) catch quit(err);
    stderr.print("\n", .{}) catch quit(err);
    bw.flush() catch quit(err);

    quit(err);
}

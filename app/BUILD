load("@rules_cc//cc:defs.bzl", "cc_binary")

# @coworkers_project//:some-lib
# cc_binary(
# 	name = "cv_test",
# 	srcs = ["cv_test.cc"],
#     deps = ["@opencv//:opencv"],
# 	)

cc_binary(
	name = "v_frontend",
	srcs = ["VisualFrontend.cpp"],
    copts = [""],
    includes = ["*.hpp"],
    deps = ["@opencv//:opencv", "//src:v_frontend_lib"],
	)
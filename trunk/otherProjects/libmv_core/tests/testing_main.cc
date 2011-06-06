// Copyright (c) 2009 libmv authors.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to
// deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
// sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

// All libmv tests should use this main unless there is a good reason not to.

#include "testing.h"
#include "test_data_sets.h"
#include <fstream>

// The following lines pull in the real gtest *.cc files.
#include "gtest/src/gtest.cc"
#include "gtest/src/gtest-death-test.cc"
#include "gtest/src/gtest-filepath.cc"
#include "gtest/src/gtest-port.cc"
#include "gtest/src/gtest-printers.cc"
#include "gtest/src/gtest-test-part.cc"
#include "gtest/src/gtest-typed-test.cc"

DEFINE_string(test_tmpdir, "/tmp", "Dir to use for temp files");

int main(int argc, char **argv)
{
  std::ofstream Out("log.txt");
  std::clog.rdbuf(Out.rdbuf());
  testing::InitGoogleTest(&argc, argv);
  google::ParseCommandLineFlags(&argc, &argv, true);
  //google::InitGoogleLogging(argv[0]);

  return RUN_ALL_TESTS();

  // TODO(keir): Make a test_tmpdir temp directory, and delete it after running
  // the tests. This involves making a portable temporary directory function.
}


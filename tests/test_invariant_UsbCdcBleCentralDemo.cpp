#include <gtest/gtest.h>
#include <string>
#include <vector>
#include <cstring>
#include <fstream>
#include <cstdlib>

class SecurityTest : public ::testing::TestWithParam<std::string> {};

TEST_P(SecurityTest, BufferReadsNeverExceedDeclaredLength) {
    // Invariant: Buffer reads never exceed the declared length
    std::string payload = GetParam();
    
    // Create a temporary test file with the payload
    std::string test_filename = "test_input.bin";
    std::ofstream test_file(test_filename, std::ios::binary);
    test_file.write(payload.c_str(), payload.size());
    test_file.close();
    
    // Execute the actual production code with the payload
    std::string command = "./UsbCdcBleCentralDemo " + test_filename;
    int result = system(command.c_str());
    
    // Clean up test file
    remove(test_filename.c_str());
    
    // Assert no crash occurred (exit code should be normal)
    EXPECT_NE(result, 139) << "Segmentation fault detected - buffer overflow likely occurred";
    EXPECT_NE(result, 134) << "Abort detected - memory corruption likely occurred";
}

INSTANTIATE_TEST_SUITE_P(
    AdversarialInputs,
    SecurityTest,
    ::testing::Values(
        // Exact exploit case: 2KB input when buffer likely expects < 256 bytes
        std::string(2048, 'A'),
        // Boundary case: exactly 256 bytes (common buffer size)
        std::string(256, 'B'),
        // Valid input: normal sized input
        std::string(64, 'C'),
        // Null-terminator overflow case
        std::string(255, 'D') + std::string(1, '\0') + std::string(100, 'E')
    )
);

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
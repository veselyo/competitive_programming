#include <iostream>
#include <vector>
#include <cassert>
#include <random>
#include <unordered_set>
#include "solution.h"
#include "harmonic.h"

using namespace solution;

#define CHECK_GREEDY(i,m,n,mat,opt)                                   \
    do {                                                              \
        int res = solve(m, n, mat);                                   \
        std::cout << "Test " << i << ": Result: " << res << " (opt: " << opt << ")" << std::endl;                   \
        int bound = static_cast<int>(std::ceil(                       \
                     (opt) * harmonic_upper_bound((m)*(n)) ));        \
        assert(res <= bound);                                         \
    } while (0)

    
void test_all_false() {
    int m = 5, n = 5;
    std::vector<std::vector<bool>> matrix = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}
    };
    CHECK_GREEDY(1, m, n, matrix, /*OPT=*/0);
}

void test_single_false1() {
    int m = 1, n = 1;
    std::vector<std::vector<bool>> matrix = {
        {0}
    };
    CHECK_GREEDY(2, m, n, matrix, /*OPT=*/0);
}

void test_single_false2() {
    int m = 4, n = 3;
    std::vector<std::vector<bool>> matrix = {
        {1, 1, 0},
        {1, 1, 1},
        {1, 1, 1},
        {1, 1, 1},
    };
    CHECK_GREEDY(3, m, n, matrix, /*OPT=*/2);
}

void test_all_true() {
    int m = 5, n = 3;
    std::vector<std::vector<bool>> matrix = {
        {1, 1, 1},
        {1, 1, 1},
        {1, 1, 1},
        {1, 1, 1},
        {1, 1, 1}
    };
    CHECK_GREEDY(4, m, n, matrix, /*OPT=*/1);
}

void test_single_true1() {
    int m = 1, n = 1;
    std::vector<std::vector<bool>> matrix = {
        {1}
    };
    CHECK_GREEDY(5, m, n, matrix, /*OPT=*/1);
}

void test_single_true2() {
    int m = 4, n = 6;
    std::vector<std::vector<bool>> matrix = {
        {0, 0, 0, 0, 0, 1},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0},
    };
    CHECK_GREEDY(6, m, n, matrix, /*OPT=*/1);
}

void test_column() {
    int m = 6, n = 2;
    std::vector<std::vector<bool>> matrix = {
        {1, 0},
        {1, 0},
        {1, 0},
        {1, 0},
        {1, 0},
        {1, 0}
    };
    CHECK_GREEDY(7, m, n, matrix, /*OPT=*/1);
}

void test_row() {
    int m = 3, n = 5;
    std::vector<std::vector<bool>> matrix = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {1, 1, 1, 1, 1},
    };
    CHECK_GREEDY(8, m, n, matrix, /*OPT=*/1);
}

void test_block() {
    int m = 7, n = 5;
    std::vector<std::vector<bool>> matrix = {
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 1, 1},
        {0, 0, 1, 1, 1},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}
    };
    CHECK_GREEDY(9, m, n, matrix, /*OPT=*/1);
}

void test_multiple_blocks() {
    int m = 6, n = 6;
    std::vector<std::vector<bool>> matrix = {
        {1, 1, 0, 0, 0, 0},
        {1, 1, 0, 0, 0, 0},
        {0, 0, 0, 1, 1, 0},
        {0, 0, 0, 1, 1, 0},
        {1, 1, 0, 0, 0, 0},
        {1, 1, 0, 0, 0, 0}
    };
    CHECK_GREEDY(10, m, n, matrix, /*OPT=*/3);
}

void test_complex_case1() {
    int m = 7, n = 6;
    std::vector<std::vector<bool>> matrix = {
        {1, 1, 0, 0, 0, 0},
        {1, 1, 0, 0, 0, 0},
        {0, 0, 1, 1, 1, 0},
        {0, 0, 1, 1, 1, 0},
        {1, 1, 0, 0, 0, 0},
        {1, 1, 0, 0, 0, 0},
        {1, 1, 0, 0, 0, 0}
    };
    CHECK_GREEDY(11, m, n, matrix, /*OPT=*/2);
}

void test_complex_case2() {
    int m = 3, n = 3;
    std::vector<std::vector<bool>> matrix = {
        {1, 0, 1},
        {0, 1, 0},
        {1, 0, 1}
    };
    CHECK_GREEDY(12, m, n, matrix, /*OPT=*/3);
}

void test_complex_case3() {
    int m = 8, n = 6;
    std::vector<std::vector<bool>> matrix = {
        {1, 0, 0, 0, 0, 0},
        {1, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 1, 1, 0, 0, 1},
        {0, 1, 0, 1, 1, 0},
        {1, 0, 1, 0, 0, 1},
        {1, 0, 1, 0, 0, 1}
    };
    CHECK_GREEDY(13, m, n, matrix, /*OPT=*/5);
}

void test_empty_matrix() {
    int m = 0, n = 0;
    std::vector<std::vector<bool>> matrix;
    CHECK_GREEDY(14, m, n, matrix, /*OPT=*/0);
}

void test_single_row_alternating() {
    int m = 1, n = 7;
    std::vector<std::vector<bool>> matrix = {
        {1, 0, 1, 0, 1, 0, 1}
    };
    CHECK_GREEDY(15, m, n, matrix, /*OPT=*/4);
}

void test_single_column_alternating() {
    int m = 6, n = 1;
    std::vector<std::vector<bool>> matrix = {
        {1}, {0}, {1}, {0}, {1}, {0}
    };
    CHECK_GREEDY(16, m, n, matrix, /*OPT=*/3);
}

void test_checkerboard_2x2() {
    int m = 2, n = 2;
    std::vector<std::vector<bool>> matrix = {
        {1, 0},
        {0, 1}
    };
    CHECK_GREEDY(17, m, n, matrix, /*OPT=*/2);
}

void test_checkerboard_4x4() {
    int m = 4, n = 4;
    std::vector<std::vector<bool>> matrix = {
        {1, 0, 1, 0},
        {0, 1, 0, 1},
        {1, 0, 1, 0},
        {0, 1, 0, 1}
    };
    CHECK_GREEDY(18, m, n, matrix, /*OPT=*/4);
}

void test_border_ring() {
    int m = 3, n = 3;
    std::vector<std::vector<bool>> matrix = {
        {1, 1, 1},
        {1, 0, 1},
        {1, 1, 1}
    };
    CHECK_GREEDY(19, m, n, matrix, /*OPT=*/2);
}

void test_cross_pattern() {
    int m = 3, n = 3;
    std::vector<std::vector<bool>> matrix = {
        {0, 1, 0},
        {1, 1, 1},
        {0, 1, 0}
    };
    CHECK_GREEDY(20, m, n, matrix, /*OPT=*/3);
}

void test_inner_3x3_block() {
    int m = 5, n = 5;
    std::vector<std::vector<bool>> matrix = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0}
    };
    CHECK_GREEDY(21, m, n, matrix, /*OPT=*/1);
}

void test_main_diagonal_4x4() {
    int m = 4, n = 4;
    std::vector<std::vector<bool>> matrix = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}
    };
    CHECK_GREEDY(22, m, n, matrix, /*OPT=*/4);
}

void test_size_mismatch_smaller() {
    int m = 3, n = 3;
    std::vector<std::vector<bool>> matrix = { {1,0,1}, {0,1,0} };
    bool threw = false;
    try {
        int res = solve(m, n, matrix);
        assert(res >= 0);
    } catch (const std::exception& e) {
        threw = true;
    }
    assert (threw);
    cout << "Test 23: Size mismatch (smaller) passed." << endl;
}

void test_size_mismatch_ragged() {
    int m = 3, n = 3;
    std::vector<std::vector<bool>> matrix = {
        {1,0,1},
        {1},
        {0,1,0}
    };
    bool threw = false;
    try {
        int res = solve(m, n, matrix);
        assert(res >= 0);
    } catch (const std::exception& e) {
        threw = true;
    }
    assert (threw);
    cout << "Test 24: Size mismatch (ragged) passed." << endl;
}

void test_negative_dims() {
    int m = -4, n = 5;
    std::vector<std::vector<bool>> matrix = { {1,1,1,1,1} };
    bool threw = false;
    try {
        int res = solve(m, n, matrix);
        assert(res >= 0);
    } catch (const std::exception& e) {
        threw = true;
    }
    assert (threw);
    cout << "Test 25: Negative dimensions passed." << endl;
}

void test_empty_vector_positive_dims() {
    int m = 4, n = 4;
    std::vector<std::vector<bool>> matrix;
    bool threw = false;
    try {
        int res = solve(m, n, matrix);
        assert(res >= 0);
    } catch (const std::exception& e) {
        threw = true;
    }
    assert (threw);
    cout << "Test 26: Empty vector with positive dimensions passed." << endl;
}


int main() {
    test_all_false();
    test_single_false1();
    test_single_false2();
    test_all_true();
    test_single_true1();
    test_single_true2();
    test_column();
    test_row();
    test_block();
    test_multiple_blocks();
    test_complex_case1();
    test_complex_case2();
    test_complex_case3();
    test_empty_matrix();
    test_single_row_alternating();
    test_single_column_alternating();
    test_checkerboard_2x2();
    test_checkerboard_4x4();
    test_border_ring();
    test_cross_pattern();
    test_inner_3x3_block();
    test_main_diagonal_4x4();
    test_size_mismatch_smaller();
    test_size_mismatch_ragged();
    test_negative_dims();
    test_empty_vector_positive_dims();

    cout << "All tests passed." << endl;
    return 0;
}
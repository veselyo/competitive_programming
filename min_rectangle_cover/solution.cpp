#include "solution.h"
#include <vector>
#include <algorithm>
#include <stdexcept>

using namespace std;

namespace solution {
    struct Rect{
        int r1, c1, r2, c2;
    };

    bool matrix_has_ones(const vector<vector<bool>>& matrix){
        for (const auto& row : matrix){
            for (bool val : row){
                if (val) return true;
            }
        }
        return false;
    }

    static void build_pref(const vector<vector<bool>>& matrix,
                           vector<vector<int>>& pref){
        int m = matrix.size();
        int n = matrix[0].size();

        for (int i = 0; i < m; ++i) {
            int row = 0;
            for (int j = 0; j < n; ++j) {
                row += matrix[i][j];
                pref[i+1][j+1] = pref[i][j+1] + row;
            }
        }
    }

    static int sum_rect(const vector<vector<int>>& pref,
                        int r1,int c1,int r2,int c2){
        return pref[r2+1][c2+1] -
               pref[r1][c2+1] -
               pref[r2+1][c1] +
               pref[r1][c1];
    }

    int solve(int m, int n, vector<vector<bool>> matrix){
        if (m == 0 || n == 0) return 0;
        if (m < 0 || n < 0) {
            throw std::invalid_argument("m < 0 or n < 0");
        }
        if (m != matrix.size()) {
            throw std::invalid_argument("m != matrix.size()");
        }
        for (int i = 0; i < m; ++i) {
            if ((int)matrix[i].size() != n) {
              throw std::invalid_argument("row “i” length != n");
            }
        }
        if (!matrix_has_ones(matrix)) return 0;

        vector<Rect> rects;
        rects.reserve((m * (m + 1) / 2) * (n * (n + 1) / 2));
        for (int r1 = 0; r1 < m; ++r1){
            for (int c1 = 0; c1 < n; ++c1){
                for (int r2 = r1; r2 < m; ++r2){
                    for (int c2 = c1; c2 < n; ++c2){
                        rects.push_back({r1, c1, r2, c2});
                    }
                }
            }
        }

        vector<vector<int>> pref(m + 1, vector<int>(n + 1, 0));

        int flips = 0;
        while (matrix_has_ones(matrix)) {
            build_pref(matrix, pref);
            int best_idx   = -1;
            int best_cover = -1;

            for (size_t i = 0; i < rects.size(); ++i) {
                Rect& curr_rect = rects[i];

                if (!matrix[curr_rect.r1][curr_rect.c1]) continue;

                int cover = sum_rect(pref, curr_rect.r1, curr_rect.c1,
                                     curr_rect.r2, curr_rect.c2);

                if (cover > best_cover) {
                    best_cover = cover;
                    best_idx = static_cast<int>(i);
                }
            }

            if (best_cover <= 0 || best_idx < 0 || best_idx >= rects.size()) {
                throw std::runtime_error("Invalid best_cover or best_idx");
            }

            Rect& best_rect = rects[best_idx];
            for (int r = best_rect.r1; r <= best_rect.r2; ++r){
                for (int c = best_rect.c1; c <= best_rect.c2; ++c){
                    matrix[r][c] = !matrix[r][c];
                }
            }

            ++flips;
        }

        return flips;
    }
}   
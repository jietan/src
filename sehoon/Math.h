#ifndef SEHOON_MATH_H
#define SEHOON_MATH_H

#include <vector>
#include <utility>

namespace sehoon {
    namespace math {
        static double solveQuadraticEquation(double a, double b, double c);
        template <class A, class V>
        A argmin(const std::vector< std::pair<A, V> >& lists);

        //////////////////////////////////////////////////
        //
        static double solveQuadraticEquation(double a, double b, double c) {
            return (-b - sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
        }

        template <class A, class V>
        A argmin(const std::vector< std::pair<A, V> >& lists) {
            A minArgument = lists[0].first;
            V minValue = lists[0].second;

            for (size_t i = 1; i < lists.size(); i++) {
                if (minValue > lists[i].second) {
                    minValue = lists[i].second;
                    minArgument = lists[i].first;
                }
            }
            return minArgument;
        }

        
    } // namespace math
} // namespace sehoon

#endif // #ifndef SEHOON_MATH_H


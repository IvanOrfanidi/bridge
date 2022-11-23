#include <cmath>
#include <limits>
#include <type_traits>

namespace drone {
namespace math {

template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
bool isEqual(const T& x, const T& y) {
    return std::fabs(x - y) < std::numeric_limits<T>::epsilon();
}

template<typename T, std::enable_if_t<std::is_floating_point<T>::value, bool> = true>
bool isEqual(const T& x, const T& y, const T delta) {
    return std::fabs(x - y) < delta;
}

template<typename T, std::enable_if_t<!std::is_floating_point<T>::value, bool> = true>
bool isEqual(const T& x, const T& y) {
    return x == y;
}

} // namespace math
} // namespace drone
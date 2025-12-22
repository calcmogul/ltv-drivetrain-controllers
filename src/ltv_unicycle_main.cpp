#include <wpi/math/controller/ltv_unicycle_controller.hpp>

int main() {
  constexpr double dt = 0.05;

  wpi::math::LTVUnicycleController unicycle_controller{dt};
  unicycle_controller.calculate({{0.0, 0.0}, 0.0}, {{0.0, 0.0}, 0.0}, 1.0, 1.0);
}

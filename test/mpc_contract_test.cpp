#include <wolf_controller/mpc_contract.h>

#include <iostream>
#include <limits>
#include <stdexcept>

using namespace wolf_controller::mpc_contract;

static void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}

int main() {
  const std::vector<std::string> feet{"lf_foot", "lh_foot", "rf_foot", "rh_foot"};
  require(kStateSize == 24 && kInputSize == 24 && kForceCount == 12,
          "Centroidal input/state layout must be 24/24 with joint velocities at 12");
  require(encodeMode({true, false, false, false}) == 8, "LF gait bit");
  require(encodeMode({false, true, false, false}) == 2, "LH gait bit");
  require(encodeMode({false, false, true, false}) == 4, "RF gait bit");
  require(encodeMode({false, false, false, true}) == 1, "RH gait bit");
  require(encodeMode({true, false, false, true}) == 9, "LF/RH diagonal stance");
  require(encodeMode({false, true, true, false}) == 6, "LH/RF diagonal stance");

  require(canonicalLegOrder({"rh_foot", "rf_foot", "lf_foot", "lh_foot"}) == feet,
          "Canonical contact order must not depend on input traversal order");
  std::array<bool, 4> state{};
  require(reorderContacts(feet, {"rh_foot", "rf_foot", "lf_foot", "lh_foot"},
                          {true, false, true, false}, state), "Reordered command rejected");
  require(state == std::array<bool, 4>{{true, false, false, true}}, "Named contact mapping");
  const auto accepted = state;
  require(!reorderContacts(feet, {"rh_foot", "rf_foot", "lf_foot", "lf_foot"},
                           {false, false, false, false}, state), "Duplicate names accepted");
  require(!reorderContacts(feet, {"rh_foot", "rf_foot", "lf_foot", "unknown"},
                           {false, false, false, false}, state), "Unknown name accepted");
  require(!reorderContacts(feet, feet, {true}, state), "Partial contact vector accepted");
  require(state == accepted, "Invalid commands changed the accepted schedule");

  require(!contactReferenceExpired(false, true, 10.0, 0.25), "WPG must not be stopped by old MPC data");
  require(!contactReferenceExpired(true, false, 10.0, 0.25), "Legacy EXT has no adopted contact stream");
  require(!contactReferenceExpired(true, true, 0.1, 0.25), "Fresh command rejected");
  require(contactReferenceExpired(true, true, 0.251, 0.25), "Stale active MPC command did not stop");
  require(contactReferenceExpired(true, true, -1.0, 0.25), "Clock reversal not rejected");
  require(contactReferenceExpired(true, true, std::numeric_limits<double>::quiet_NaN(), 0.25),
          "Non-finite age not rejected");
  require(allowMpcStart(true, true, true, true, true), "Ready active controller rejected MPC start");
  require(!allowMpcStart(false, true, true, true, true), "Inactive controller accepted MPC start");
  require(!allowMpcStart(true, false, false, true, true), "Missing first contact command accepted");
  require(!allowMpcStart(true, true, false, true, true), "Unconsumed contact buffer allowed MPC entry");
  require(!allowMpcStart(true, true, true, false, true), "Expired contact command accepted");
  require(!allowMpcStart(true, true, true, true, false), "Missing or stale individual reference accepted");
  require(referenceWatchdogAction(false, true, true, false, true) == WatchdogAction::CancelPending,
          "A deferred EXT request survived expiry before activation");
  require(referenceWatchdogAction(true, false, true, true, false) == WatchdogAction::StopActive,
          "An active EXT command was not stopped after one reference expired");
  require(referenceWatchdogAction(false, false, true, false, false) == WatchdogAction::None,
          "Stopped WPG was treated as active MPC");
  require(referenceWatchdogAction(true, true, false, false, false) == WatchdogAction::None,
          "A legacy EXT client without adopted MPC contacts changed behavior");
  require(referenceWatchdogAction(true, true, true, true, true) == WatchdogAction::None,
          "Fresh references triggered the watchdog");
  std::cout << "MPC contract tests passed\n";
}

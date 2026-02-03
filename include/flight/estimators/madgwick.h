#pragma once

#include "flight/estimators/estimators.h"

namespace flight::estimators {

class MadgwickEstimator final : public IStateEstimator {
 public:
  bool Initialize() override;
  EstimatorOutput Update(const EstimatorInput& input) override;

 private:
  EstimatorOutput state_{};
};

}  // namespace flight::estimators

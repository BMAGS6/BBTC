#include "bbtc/diagnostics.h"

const char*
bbtc_ib_termination_string(const bbtc_ib_termination_e termination)
{
    switch (termination)
    {
        case BBTC_IB_TERMINATION_NOT_RUN:
            return "simulation not run";

        case BBTC_IB_TERMINATION_MUZZLE_EXIT:
            return "projectile reached muzzle exit";

        case BBTC_IB_TERMINATION_NO_IGNITION:
            return "ignition did not occur";

        case BBTC_IB_TERMINATION_PROJECTILE_NOT_STARTED:
            return "projectile did not begin moving";

        case BBTC_IB_TERMINATION_PROJECTILE_STOPPED_BEFORE_MUZZLE_EXIT:
            return "projectile stopped before muzzle exit";

        case BBTC_IB_TERMINATION_TIME_GUARD_REACHED:
            return "caller time guard reached";

        case BBTC_IB_TERMINATION_PRESSURE_GUARD_REACHED:
            return "caller pressure guard reached";

        case BBTC_IB_TERMINATION_STEP_GUARD_REACHED:
            return "caller step guard reached";

        case BBTC_IB_TERMINATION_NUMERICAL_FAILURE:
            return "numerical failure";

        default:
            return "unknown BBTC internal-ballistics termination";
    }
}

const char*
bbtc_warning_flag_string(const bbtc_warning_flag_e flag)
{
    switch (flag)
    {
        case BBTC_WARNING_NONE:
            return "no warning reported";

        case BBTC_WARNING_HISTORY_TRUNCATED:
            return "requested history was truncated";

        case BBTC_WARNING_ENERGY_RESIDUAL_EXCEEDED:
            return "energy-accounting residual tolerance exceeded";

        case BBTC_WARNING_INCOMPLETE_BURN_AT_MUZZLE_EXIT:
            return "propellant burn incomplete at muzzle exit";

        case BBTC_WARNING_FALLBACK_APPROXIMATION_USED:
            return "fallback approximation used";

        case BBTC_WARNING_REDUCED_EVENT_LOCATION_ACCURACY:
            return "event located with reduced accuracy";

        case BBTC_WARNING_DATA_EXTRAPOLATED:
            return "data record extrapolated";

        default:
            return "unknown or combined BBTC warning flag";
    }
}

const char*
bbtc_applicability_flag_string(const bbtc_applicability_flag_e flag)
{
    switch (flag)
    {
        case BBTC_APPLICABILITY_NONE_REPORTED:
            return "no applicability limitation reported";

        case BBTC_APPLICABILITY_OUTSIDE_CALIBRATION_DOMAIN:
            return "outside documented calibration domain";

        case BBTC_APPLICABILITY_MODEL_COMBINATION_UNVALIDATED:
            return "model combination lacks experimental validation";

        case BBTC_APPLICABILITY_ASSUMPTIONS_MATERIALLY_STRESSED:
            return "model assumptions materially stressed";

        case BBTC_APPLICABILITY_DATA_PROVENANCE_UNKNOWN:
            return "data provenance unknown";

        case BBTC_APPLICABILITY_REQUESTED_EFFECT_APPROXIMATED:
            return "requested physical effect approximated";

        default:
            return "unknown or combined BBTC applicability flag";
    }
}

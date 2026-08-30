/**
 * @file status.c
 */
#include "bbtc/status.h"

const char*
bbtc_status_string(const bbtc_status_e status)
{
    switch (status)
    {
        case BBTC_STATUS_SUCCESS:
            return "success";

        case BBTC_STATUS_INVALID_ARGUMENT:
            return "invalid argument";

        case BBTC_STATUS_NONFINITE_INPUT:
            return "non-finite input";

        case BBTC_STATUS_OUTSIDE_DOMAIN:
            return "value outside mathematical domain";

        case BBTC_STATUS_INCONSISTENT_CONFIGURATION:
            return "inconsistent geometry or configuration";

        case BBTC_STATUS_UNSUPPORTED_MODEL_OR_OPTION:
            return "unsupported model or option";

        case BBTC_STATUS_INSUFFICIENT_STORAGE:
            return "insufficient caller-provided storage";

        case BBTC_STATUS_NUMERICAL_FAILURE:
            return "numerical failure";

        case BBTC_STATUS_ITERATION_LIMIT:
            return "iteration or step limit reached";

        case BBTC_STATUS_INTERNAL_INVARIANT_FAILURE:
            return "internal invariant failure";

        case BBTC_STATUS_NAN_INPUT:
            return "not-a-number input";

        default:
            return "unknown BBTC status";
    }
}

#include "firmware/job_ingress_state.hpp"

namespace firmware
{

    static JobIngressState g_job_ingress_state;

    JobIngressState &get_job_ingress_state()
    {
        return g_job_ingress_state;
    }

}

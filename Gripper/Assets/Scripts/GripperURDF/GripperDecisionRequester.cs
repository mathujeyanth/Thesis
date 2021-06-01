using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.MLAgents
{
    public class GripperDecisionRequester : DecisionRequester
    {
        [ReadOnly] public bool grasped = false;
        protected override bool ShouldRequestDecision(DecisionRequestContext context)
        {
            return context.AcademyStepCount % DecisionPeriod == 0 && grasped;
        }
        protected override bool ShouldRequestAction(DecisionRequestContext context)
        {
            return TakeActionsBetweenDecisions && grasped;
        }
    }
}

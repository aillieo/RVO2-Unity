
namespace RVO
{
    using Unity.Mathematics;

    /// <summary>
    /// Defines an agent in the simulation.
    /// </summary>
    public class Agent
    {
        public int id;

        public float2 position;
        public float2 prefVelocity;
        public float2 velocity;
        public int maxNeighbors;
        public float maxSpeed;
        public float neighborDist;
        public float radius;
        public float timeHorizon;
        public float timeHorizonObst;
        public float weight;
        public float2 newVelocity;

        public Agent()
        {

        }

        public Agent(AgentData agentData)
        {
            id = agentData.id;
            maxNeighbors = agentData.maxNeighbors;
            maxSpeed = agentData.maxSpeed;
            neighborDist = agentData.neighborDist;
            position = agentData.position;
            radius = agentData.radius;
            timeHorizon = agentData.timeHorizon;
            timeHorizonObst = agentData.timeHorizonObst;
            velocity = agentData.velocity;
            weight = agentData.weight;
        }
    }
}

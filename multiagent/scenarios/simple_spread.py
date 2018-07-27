import numpy as np
from multiagent.core import World, Agent, Landmark
from multiagent.scenario import BaseScenario


class Scenario(BaseScenario):
    def make_world(self):
        world = World()
        # set any world properties first
        world.dim_c = 2
        num_agents = 3
        num_landmarks = 3
        # add agents
        world.agents = [Agent() for i in range(num_agents)]
        for i, agent in enumerate(world.agents):
            agent.name = 'agent %d' % i
            agent.collide = True
            agent.silent = True
            agent.size = 0.15
        # add landmarks
        world.landmarks = [Landmark() for i in range(num_landmarks)]
        for i, landmark in enumerate(world.landmarks):
            landmark.name = 'landmark %d' % i
            landmark.collide = False
            landmark.movable = False
        # make initial conditions
        self.reset_world(world)
        return world

    def reset_world(self, world):
        # random properties for agents
        for i, agent in enumerate(world.agents):
            agent.color = np.array([0.35, 0.35, 0.85])
        # random properties for landmarks
        for i, landmark in enumerate(world.landmarks):
            landmark.color = np.array([0.25, 0.25, 0.25])
        # set random initial states
        for agent in world.agents:
            agent.state.p_pos = np.random.uniform(-1, +1, world.dim_p)
            agent.state.p_vel = np.zeros(world.dim_p)
            agent.state.c = np.zeros(world.dim_c)
        for i, landmark in enumerate(world.landmarks):
            landmark.state.p_pos = np.random.uniform(-1, +1, world.dim_p)
            landmark.state.p_vel = np.zeros(world.dim_p)

    def benchmark_data(self, agent, world):
        rew = 0
        collisions = 0
        occupied_landmarks = 0
        min_dists = 0
        for l in world.landmarks:
            dists = [np.sqrt(np.sum(np.square(a.state.p_pos - l.state.p_pos))) for a in world.agents]
            min_dists += min(dists)
            rew -= min(dists)
            if min(dists) < 0.1:
                occupied_landmarks += 1
        if agent.collide:
            for a in world.agents:
                if self.is_collision(a, agent):
                    rew -= 1
                    collisions += 1
        return (rew, collisions, min_dists, occupied_landmarks)


    def is_collision(self, agent1, agent2):
        delta_pos = agent1.state.p_pos - agent2.state.p_pos
        dist = np.sqrt(np.sum(np.square(delta_pos)))
        dist_min = agent1.size + agent2.size
        return True if dist < dist_min else False


    def reward(
            self,
            agent,
            world,
            terminal,
            done,
            agents_positions,
    ):
        rew_sum_dist = 0
        rew_terminate = 0
        rew_goal = 0
        rew_out_goal = 0
        rew_collision = 0

        for t in terminal:
            if t:
                rew_terminate -= 10

        agents2land_dist_pre = []
        for land in world.landmarks:
            land_pos = land.state.p_pos
            for agent_pos in agents_positions:
                land_agent_dist_ = land_pos - agent_pos
                land_agent_dist = np.sqrt(np.sum(np.square(land_agent_dist_)))
                agents2land_dist_pre.append(land_agent_dist)

        x_temp = [agents2land_dist_pre[0] + agents2land_dist_pre[4] + agents2land_dist_pre[8],
                           agents2land_dist_pre[0] + agents2land_dist_pre[5] + agents2land_dist_pre[7],
                           agents2land_dist_pre[1] + agents2land_dist_pre[3] + agents2land_dist_pre[8],
                           agents2land_dist_pre[1] + agents2land_dist_pre[5] + agents2land_dist_pre[6],
                           agents2land_dist_pre[2] + agents2land_dist_pre[3] + agents2land_dist_pre[7],
                           agents2land_dist_pre[2] + agents2land_dist_pre[4] + agents2land_dist_pre[6]]

        min_dist_pre = min([agents2land_dist_pre[0] + agents2land_dist_pre[4] + agents2land_dist_pre[8],
                           agents2land_dist_pre[0] + agents2land_dist_pre[5] + agents2land_dist_pre[7],
                           agents2land_dist_pre[1] + agents2land_dist_pre[3] + agents2land_dist_pre[8],
                           agents2land_dist_pre[1] + agents2land_dist_pre[5] + agents2land_dist_pre[6],
                           agents2land_dist_pre[2] + agents2land_dist_pre[3] + agents2land_dist_pre[7],
                           agents2land_dist_pre[2] + agents2land_dist_pre[4] + agents2land_dist_pre[6]])
        # print('\nagents2land_dist_pre :')
        # print(agents2land_dist_pre.__str__())
        print('\nmin of min_dist_pre :')
        print(min_dist_pre.__str__())
        print('\nindex of the min distance to each landmark')
        print(x_temp.index(min_dist_pre))

        print('################################################################')
        agents2land_dist = []
        for land in world.landmarks:
            land_pos = land.state.p_pos
            for agent in world.agents:
                agent_pos = agent.state.p_pos
                land_agent_dist_ = land_pos - agent_pos
                land_agent_dist = np.sqrt(np.sum(np.square(land_agent_dist_)))
                agents2land_dist.append(land_agent_dist)
        min_dist = min([agents2land_dist[0] + agents2land_dist[4] +
                           agents2land_dist[8],
                           agents2land_dist[0] + agents2land_dist[5] +
                           agents2land_dist[7],
                           agents2land_dist[1] + agents2land_dist[3] +
                           agents2land_dist[8],
                           agents2land_dist[1] + agents2land_dist[5] +
                           agents2land_dist[6],
                           agents2land_dist[2] + agents2land_dist[3] +
                           agents2land_dist[7],
                           agents2land_dist[2] + agents2land_dist[4] +
                           agents2land_dist[6]])
        x_temp = [agents2land_dist[0] + agents2land_dist[4] +
                           agents2land_dist[8],
                           agents2land_dist[0] + agents2land_dist[5] +
                           agents2land_dist[7],
                           agents2land_dist[1] + agents2land_dist[3] +
                           agents2land_dist[8],
                           agents2land_dist[1] + agents2land_dist[5] +
                           agents2land_dist[6],
                           agents2land_dist[2] + agents2land_dist[3] +
                           agents2land_dist[7],
                           agents2land_dist[2] + agents2land_dist[4] +
                           agents2land_dist[6]]
        # print('\nagents2land_dist :')
        # print(agents2land_dist.__str__())
        print('\nmin min_dist:')
        print(min_dist.__str__())
        print('\nindex of the min distance to each landmark')
        print(x_temp.index(min_dist))

        agent_2land_dist_min = min_dist_pre - min_dist
        rew_sum_dist = agent_2land_dist_min

        # print('\nreward sum distance : ')
        # print(rew_sum_dist.__str__())
        print('################################################################')
        # print('################################################################')
        # print('################################################################')

        for l, sublist in enumerate(done):
            if sublist[-1] is True and sublist[-2] is False:
                rew_goal += 5

            if sublist[-1] is False and sublist[-2] is True:
                rew_out_goal -= 5

        if agent.collide:
            collision = [False] * len(world.agents)
            for i, agent in enumerate(world.agents):
                for a in world.agents:
                    if agent is a:
                        continue
                    if self.is_collision(a, agent):
                        rew_collision -= 10

        print('\nrew_collision:')
        print(rew_collision.__str__())
        print('\nrew_out_goal')
        print(rew_out_goal.__str__())
        print('\nrew_goal:')
        print(rew_goal.__str__())
        print('\nrew_sum_dist:')
        print(rew_sum_dist.__str__())
        print('\nrew_terminal')
        print(rew_terminate.__str__())
        print('################################################################')
        # print('################################################################')
        # print('################################################################')

        reward_total = rew_collision + rew_out_goal + rew_goal + rew_sum_dist + rew_terminate
        return reward_total/10.0

    def observation(self, agent, world):
        # get positions of all entities in this agent's reference frame
        entity_pos = []
        for entity in world.landmarks:  # world.entities:
            entity_pos.append(entity.state.p_pos - agent.state.p_pos)
        # entity colors
        entity_color = []
        for entity in world.landmarks:  # world.entities:
            entity_color.append(entity.color)
        # communication of all other agents
        comm = []
        other_pos = []
        for other in world.agents:
            if other is agent: continue
            comm.append(other.state.c)
            other_pos.append(other.state.p_pos - agent.state.p_pos)
        return np.concatenate([agent.state.p_vel] + [agent.state.p_pos] + entity_pos + other_pos + comm)

    def done(self, agent, world):
        distance_to_each_landmark = [
            np.sqrt(np.sum(np.square(a.state.p_pos - agent.state.p_pos)))
            for a in world.landmarks
        ]
        dist_min = [
            a.size + agent.size
            for a in world.landmarks
        ]
        done__ = [
            True if distance_to_each_landmark[i] < dist_min[i] else False
            for i in range(len(distance_to_each_landmark))
        ]
        return True if any(done__) else False

        # distance_to_each_landmark = [
        #     np.sqrt(np.sum(np.square(a.state.p_pos - agent.state.p_pos)))
        #     for a in world.landmarks
        # ]
        # return True if max(distance_to_each_landmark) < 0.1 else False

    def terminal(self, agent):
        x = abs(agent.state.p_pos[0])
        y = abs(agent.state.p_pos[1])
        if (x > 1.0 or y > 1.0):
            return True
        return False

    def collision_detection(self, world):
        collision = [False] * len(world.agents)
        for i, agent in enumerate(world.agents):
            for a in world.agents:
                if agent is a:
                    continue
                if self.is_collision(a, agent):
                    collision[i] = True

        return collision

    def get_in_get_out_from_landmark(self, done):
        agents_getin_getout = [False] * len(done)
        for l, sublist in enumerate(done):
            if sublist[-1] is False and sublist[-2] is True:
                agents_getin_getout[l] = True
        return agents_getin_getout

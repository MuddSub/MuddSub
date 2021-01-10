from gym.envs.registration import register

register(
    id='MuddSub-v0',
    entry_point='gym_MuddSub.envs:MuddSubEnv',
)

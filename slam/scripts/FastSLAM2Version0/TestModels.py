import random
import numpy as np
import matplotlib.pyplot as plt
import argparse

def sample_normal_distribution(variance, n=12):
    '''Sample a normal distribution with the given variance'''
    stdev = variance ** 0.5
    s = 0
    for i in range(n):
        s += random.uniform(-stdev, stdev)
    return 0.5 * s

def sample_motion_model_velocity(u_t, prev_x_t, sample=sample_normal_distribution, alphas=[1] * 6, delta_t=0.01):
    v, omega = u_t
    x, y, theta = prev_x_t
    alpha_1, alpha_2, alpha_3, alpha_4, alpha_5, alpha_6 = alphas
    v_hat = v + sample(alpha_1 * v ** 2 + alpha_2 * omega ** 2)
    omega_hat = omega + sample(alpha_3 * v ** 2 + alpha_4 * omega ** 2)
    gamma_hat = sample(alpha_5 * v ** 2 + alpha_6 * omega ** 2)

    # Make sure omega is never 0
    if -1e-10 <= omega_hat <= 1e-10:
        omega_hat = 1e-10

    x_prime = x + (v_hat / omega_hat) * (-np.sin(theta) + np.sin(theta + omega_hat * delta_t))
    y_prime = y + (v_hat / omega_hat) * (np.cos(theta) - np.cos(theta + omega_hat * delta_t))
    theta_prime = theta + omega_hat * delta_t + gamma_hat * delta_t
    return np.array([x_prime, y_prime, theta_prime])

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('test', type=str, help='the test to run')
    args = parser.parse_args()

    print(args)
    if args.test == 'sample_normal_distribution':
        samples = [sample_normal_distribution(1) for _ in range(100000)]
        plt.hist(samples, bins=101, range=(-3, 3))
        plt.show()
    elif args.test == 'sample_motion_model_velocity':
        initial_pose = np.array([0, 0, 0])
        control = np.array([10, 1])
        alphas = [1e-2, 1e-2, 1e-5, 1e-5, 1e-5, 1e-5]
        samples = np.array([sample_motion_model_velocity(control, initial_pose, delta_t=1, alphas=alphas) for _ in range(1000)])
        path = np.array([sample_motion_model_velocity(control, initial_pose, delta_t=i, alphas=[0] * 6) for i in np.linspace(0, 1, 11)])
        plt.plot(samples[:, 0], samples[:, 1], 'ro', label='sample poses')
        plt.plot(initial_pose[0], initial_pose[1], 'bo', label='initial pose')
        plt.plot(path[:, 0], path[:, 1], 'b.', linestyle='-', label='no noise path')
        plt.legend()
        plt.title('Noise parameters:' + str(alphas))
        plt.show()
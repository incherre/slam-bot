'''An implementation of an EKF for SLAM which allows adding new landmarks.'''
import numpy as np
from math import inf, sqrt, sin, cos, atan2, radians

def ekf_index(landmark_index):
    return 3 + (2 * landmark_index)

class EKF:
    '''An extended kalman filter which supports heterogeneous landmark types.'''
    def __init__(self, ekf_initial_uncertainty=0.95, ekf_odometry_noise=0.05,
                 ekf_range_noise=0.01, ekf_bearing_noise=radians(1),
                 ekf_innovation_lambda=1, ekf_landmark_threshold=5, **kwargs):
        self.landmark_types = []
        self.landmark_counts = []
        self.system_state = np.zeros((3, 1))  # Begin at x, y, theta = 0, 0, 0
        self.covariance = np.eye(3) * ekf_initial_uncertainty

        self.odometry_noise = ekf_odometry_noise
        self.range_noise = ekf_range_noise
        self.bearing_noise = ekf_bearing_noise
        self.innovation_lambda = ekf_innovation_lambda
        self.landmark_threshold = ekf_landmark_threshold

    def __assert_invariants(self):
        '''Asserts that all expected invariants are true.'''
        assert(len(self.landmark_types) == len(self.landmark_counts))

        expected_matrix_size = ekf_index(len(self.landmark_types))
        assert(self.system_state.shape == (expected_matrix_size, 1))
        assert(self.covariance.shape == (expected_matrix_size, expected_matrix_size))

        np.testing.assert_almost_equal(self.covariance, np.transpose(self.covariance))

    def pos(self):
        '''Returns the current position estimate as a vertical vector.'''
        return self.system_state[:3].copy()

    def pos_after_move(self, delta_theta, odemetry):
        '''Returns the naively computed position after the provided controls.'''
        # The model here is: turn, then move.
        theta = self.system_state[2][0] + delta_theta

        x = self.system_state[0][0] + (odemetry * cos(theta))
        y = self.system_state[1][0] + (odemetry * sin(theta))

        return np.array([[x], [y], [theta]])

    def update(self, delta_theta, odemetry, observed_landmarks):
        '''Updates the efk given the controls and observations.
           Landmarks are given in the format [(x, y, type), ...].'''
        # Naively update belief.
        self.system_state[:3] = self.pos_after_move(delta_theta, odemetry)

        self.__update_position_covariance(delta_theta, odemetry)

        associated_landmarks, new_landmarks = self.__associate_landmarks(observed_landmarks)
        
        self.__update_system_state(associated_landmarks)

        self.__add_landmarks(new_landmarks, odemetry)

        self.__assert_invariants()

    def __update_system_state(self, associated_landmarks):
        '''Update the system state with the Kalman gain for all reobserved landmarks.'''
        [pos_x], [pos_y], [pos_theta] = self.pos()

        # One component of the landmark error is dependent on the landmark pos, so this is only a partial matrix.
        partial_landmark_error = np.zeros((2, 2))
        partial_landmark_error[1][1] = self.bearing_noise

        for landmark_observation, landmark_index in associated_landmarks:
            # Mark that we have reobserved the landmark.
            self.landmark_counts[landmark_index] += 1

            # Only use landmarks that we have seen enough times.
            if self.landmark_counts[landmark_index] <= self.landmark_threshold:
                continue

            [old_landmark_x], [old_landmark_y], _ = self.__landmark(landmark_index)
            new_landmark_x, new_landmark_y, _ = landmark_observation

            # Complete the landmark error matrix. Will be overwritten by future iterations.
            partial_landmark_error[0][0] = sqrt(
                (old_landmark_x - pos_x) ** 2 + (old_landmark_y - pos_y) ** 2) * self.range_noise

            # Compute the Kalman gain and apply it to the system state.
            mmj = self.__measurement_model_jacobian(landmark_index)
            ic = self.__landmark_innovation_covariance(landmark_index, partial_landmark_error)
            kalman_gain = self.covariance @ np.transpose(mmj) @ np.linalg.inv(ic)

            landmark_deviation = (np.array([[sqrt((new_landmark_x - pos_x) ** 2 + (new_landmark_y - pos_y) ** 2)],
                                            [atan2(new_landmark_y - pos_y, new_landmark_x - pos_x) - pos_theta]]) -
                                  np.array([[sqrt((old_landmark_x - pos_x) ** 2 + (old_landmark_y - pos_y) ** 2)],
                                            [atan2(old_landmark_y - pos_y, old_landmark_x - pos_x) - pos_theta]]))

            self.system_state += kalman_gain @ landmark_deviation

    def __update_position_covariance(self, delta_theta, odemetry):
        '''Update the position covariance to account for the new uncertainty.'''
        pmj = self.__prediction_model_jacobian(odemetry)
        self.covariance[:3, :3] = ((pmj @ self.covariance[:3, :3] @ np.transpose(pmj)) +
                                   self.__control_noise(delta_theta, odemetry))

    def __prediction_model_jacobian(self, odemetry):
        '''Computes the jacobian of the prediction model, given the provided controls.'''
        _, _, [pos_theta] = self.pos()
        pmj = np.eye(3)
        pmj[0][2] = -odemetry * cos(pos_theta)
        pmj[1][2] = odemetry * sin(pos_theta)
        return pmj

    def __control_noise(self, delta_theta, odemetry):
        '''Computes the control noise matrix, given the provided controls.'''
        _, _, [pos_theta] = self.pos()
        components = [odemetry * cos(pos_theta), odemetry * sin(pos_theta), delta_theta]

        control_noise = np.zeros((3, 3))
        for i, component_one in enumerate(components):
            for j, component_two in enumerate(components):
                control_noise[i][j] = self.odometry_noise * component_one * component_two

        return control_noise

    def __associate_landmarks(self, observed_landmarks):
        '''Associates currently associated landmarks with previously associated landmarks.'''
        if len(self.landmark_types) == 0:
            return ([], observed_landmarks)

        associated_landmarks = []
        new_landmarks = []

        [pos_x], [pos_y], [pos_theta] = self.pos()

        # One component of the landmark error is dependent on the landmark pos, so this is only a partial matrix.
        partial_landmark_error = np.zeros((2, 2))
        partial_landmark_error[1][1] = self.bearing_noise

        for landmark in observed_landmarks:
            closest_index = min(range(len(self.landmark_types)), key=lambda index: self.__landmark_dist(index, landmark))
            closest_dist = self.__landmark_dist(closest_index, landmark)

            if closest_dist is inf:
                new_landmarks.append(landmark)
                continue

            [old_landmark_x], [old_landmark_y], _ = self.__landmark(closest_index)
            new_landmark_x, new_landmark_y, _ = landmark

            # Complete the landmark error matrix. Will be overwritten by future iterations.
            partial_landmark_error[0][0] = sqrt(
                (old_landmark_x - pos_x) ** 2 + (old_landmark_y - pos_y) ** 2) * self.range_noise

            # Essentially, this checks whether or not the new observation is within the uncertainty
            # ellipse of the old landmark.
            innovation = np.array([[old_landmark_x - new_landmark_x], [old_landmark_y - new_landmark_y]])
            innovation_covariance = self.__landmark_innovation_covariance(closest_index, partial_landmark_error)
            innovation_gate = np.transpose(innovation) @ np.linalg.inv(innovation_covariance) @ innovation

            if innovation_gate <= self.innovation_lambda:
                associated_landmarks.append((landmark, closest_index))
            else:
                new_landmarks.append(landmark)

        return (associated_landmarks, new_landmarks)

    def __landmark_innovation_covariance(self, landmark_index, landmark_error):
        '''Computes the innovation covariance for the landmark at the specified index.'''
        mmj = self.__measurement_model_jacobian(landmark_index)
        return ((mmj @ self.covariance @ np.transpose(mmj)) +
                (np.identity(2) @ landmark_error @ np.transpose(np.identity(2))))

    def __measurement_model_jacobian(self, landmark_index):
        '''Computes and returns the jacobian of the measurement model wrt a particular landmark.'''
        [x], [y], [theta] = self.pos()
        [lx], [ly], _ = self.__landmark(landmark_index)
        r = sqrt((x - lx) ** 2 + (y - ly) ** 2)

        mmj = np.zeros((2, ekf_index(len(self.landmark_types))))
        mmj[0][0] = (x - lx) / r
        mmj[0][1] = (y - ly) / r
        mmj[0][2] = 0
        mmj[1][0] = (ly - y) / r ** 2
        mmj[1][1] = (lx - x) / r ** 2
        mmj[1][2] = -1

        mmj[0][ekf_index(landmark_index)] = -mmj[0][0]
        mmj[0][ekf_index(landmark_index) + 1] = -mmj[0][1]
        mmj[1][ekf_index(landmark_index)] = -mmj[1][0]
        mmj[1][ekf_index(landmark_index) + 1] = -mmj[1][1]

        return mmj

    def __landmark_dist(self, landmark_index, other_landmark):
        '''Computes the distance between the saved landmark at the provided index and the provided other landmark.'''
        this_landmark_x, this_landmark_y, this_landmark_type = self.__landmark(landmark_index)
        other_landmark_x, other_landmark_y, other_landmark_type = other_landmark

        if this_landmark_type != other_landmark_type:
            return inf

        return sqrt((this_landmark_x - other_landmark_x) ** 2 + (this_landmark_y - other_landmark_y) ** 2)

    def __landmark(self, landmark_index):
        '''Gets the landmark at the provided position in the form (x, y, type).'''
        landmark_x = self.system_state[ekf_index(landmark_index)]
        landmark_y = self.system_state[ekf_index(landmark_index) + 1]
        landmark_type = self.landmark_types[landmark_index]

        return (landmark_x, landmark_y, landmark_type)

    def __resize(self, new_size):
        '''Scales up the system state and covariance matrices to the new size.'''
        old_size = ekf_index(len(self.landmark_types))
        if new_size == old_size:
            return

        assert(new_size > old_size)

        new_system_state = np.zeros((new_size, 1))
        new_system_state[:old_size, :] = self.system_state
        self.system_state = new_system_state

        new_covariance = np.zeros((new_size, new_size))
        new_covariance[:old_size, :old_size] = self.covariance
        self.covariance = new_covariance

    def __add_landmarks(self, new_landmarks, odemetry):
        '''Adds the landmarks given in the format [(x, y, type), ...] to the system state and covariance matrix.'''
        next_index = ekf_index(len(self.landmark_types))
        new_matrix_size = next_index + (2 * len(new_landmarks))

        [pos_x], [pos_y], [pos_theta] = self.pos()
        landmark_prediction_jacobian = np.array([[1, 0, -odemetry * sin(pos_theta)],
                                                 [0, 1, odemetry * cos(pos_theta)]])
        landmark_range_bearing_jacobian = np.array([[cos(pos_theta), -odemetry * sin(pos_theta)],
                                                    [sin(pos_theta), odemetry * cos(pos_theta)]])

        # One component of the landmark error is dependent on the landmark pos, so this is only a partial matrix.
        partial_landmark_error = np.zeros((2, 2))
        partial_landmark_error[1][1] = self.bearing_noise

        # Only resize once, not for every landmark, to avoid extra memory allocations and copying.
        self.__resize(new_matrix_size)

        for landmark_x, landmark_y, landmark_type in new_landmarks:
            self.landmark_types.append(landmark_type)
            self.landmark_counts.append(1)
            self.system_state[next_index] = landmark_x
            self.system_state[next_index + 1] = landmark_y

            # Complete the landmark error matrix. Will be overwritten by future iterations.
            partial_landmark_error[0][0] = sqrt(
                (landmark_x - pos_x) ** 2 + (landmark_y - pos_y) ** 2) * self.range_noise

            # Add the landmarks to the covariance matrix.
            # Landmark self-covariance
            self.covariance[next_index:next_index + 2, next_index:next_index + 2] = (
                landmark_prediction_jacobian @ self.covariance[:3, :3] @ np.transpose(landmark_prediction_jacobian) +
                landmark_range_bearing_jacobian @ partial_landmark_error @ np.transpose(landmark_range_bearing_jacobian))

            # Landmark-agent covariance
            self.covariance[:3, next_index:next_index + 2] = self.covariance[:3, :3] @ np.transpose(landmark_prediction_jacobian)
            self.covariance[next_index:next_index + 2, :3] = np.transpose(self.covariance[:3, next_index:next_index + 2])

            # All the landmark-landmark covariances
            for index in range(len(self.landmark_types) - 1):
                mat_pos_i = ekf_index(index)
                self.covariance[next_index:next_index + 2, mat_pos_i:mat_pos_i + 2] = (landmark_prediction_jacobian @
                    self.covariance[:3, mat_pos_i:mat_pos_i + 2])
            self.covariance[3:3 + (2 * (len(self.landmark_types) - 1)), next_index:next_index + 2] = np.transpose(
                self.covariance[next_index:next_index + 2, 3:3 + (2 * (len(self.landmark_types) - 1))])

            next_index += 2

class KalmanFilter:
    def __init__(self, process_variance, measurement_variance):
        self.x_est = 0.0  # Initial state
        self.P = 1.0      # Initial error covariance
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance

    def predict(self):
        # Prediction step
        self.P += self.process_variance

    def update(self, measurement):
        # Update step
        K = self.P / (self.P + self.measurement_variance)  # Kalman gain
        self.x_est += K * (measurement - self.x_est)      # Update state estimate
        self.P *= (1 - K)                                 # Update error covariance
        return self.x_est
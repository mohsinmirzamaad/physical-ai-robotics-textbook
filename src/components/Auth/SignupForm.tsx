import React, { useState } from 'react';
import styles from './styles.module.css';

interface SignupFormProps {
  onSuccess?: () => void;
}

export default function SignupForm({ onSuccess }: SignupFormProps) {
  const [formData, setFormData] = useState({
    name: '',
    email: '',
    password: '',
    background: 'beginner',
    interests: [] as string[],
    hardwareExperience: '',
    softwareExperience: '',
  });
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState('');

  const interestOptions = [
    'ROS 2',
    'Robotics',
    'Computer Vision',
    'Machine Learning',
    'Simulation',
    'Hardware Integration',
    'Autonomous Navigation',
    'Humanoid Robotics',
  ];

  const handleInterestToggle = (interest: string) => {
    setFormData((prev) => ({
      ...prev,
      interests: prev.interests.includes(interest)
        ? prev.interests.filter((i) => i !== interest)
        : [...prev.interests, interest],
    }));
  };

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setIsLoading(true);

    try {
      // TODO: Implement Better-Auth signup
      // For now, we'll use a placeholder API call
      const response = await fetch('/api/auth/signup', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(formData),
      });

      if (!response.ok) {
        throw new Error('Signup failed');
      }

      const data = await response.json();
      console.log('Signup successful:', data);

      if (onSuccess) {
        onSuccess();
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Signup failed. Please try again.');
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={styles.authContainer}>
      <div className={styles.authCard}>
        <h2>Create Your Account</h2>
        <p className={styles.subtitle}>Join the Physical AI learning community</p>

        {error && <div className={styles.error}>{error}</div>}

        <form onSubmit={handleSubmit} className={styles.form}>
          {/* Basic Info */}
          <div className={styles.formGroup}>
            <label htmlFor="name">Full Name</label>
            <input
              id="name"
              type="text"
              value={formData.name}
              onChange={(e) => setFormData({ ...formData, name: e.target.value })}
              required
              placeholder="John Doe"
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="email">Email</label>
            <input
              id="email"
              type="email"
              value={formData.email}
              onChange={(e) => setFormData({ ...formData, email: e.target.value })}
              required
              placeholder="john@example.com"
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="password">Password</label>
            <input
              id="password"
              type="password"
              value={formData.password}
              onChange={(e) => setFormData({ ...formData, password: e.target.value })}
              required
              minLength={8}
              placeholder="At least 8 characters"
            />
          </div>

          {/* Background Questionnaire */}
          <div className={styles.sectionTitle}>Tell us about your background</div>

          <div className={styles.formGroup}>
            <label htmlFor="background">Experience Level</label>
            <select
              id="background"
              value={formData.background}
              onChange={(e) => setFormData({ ...formData, background: e.target.value })}
            >
              <option value="beginner">Beginner - New to robotics</option>
              <option value="intermediate">Intermediate - Some robotics experience</option>
              <option value="advanced">Advanced - Professional experience</option>
            </select>
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="softwareExperience">Software Experience</label>
            <textarea
              id="softwareExperience"
              value={formData.softwareExperience}
              onChange={(e) => setFormData({ ...formData, softwareExperience: e.target.value })}
              placeholder="e.g., Python, C++, ROS, etc."
              rows={3}
            />
          </div>

          <div className={styles.formGroup}>
            <label htmlFor="hardwareExperience">Hardware Experience</label>
            <textarea
              id="hardwareExperience"
              value={formData.hardwareExperience}
              onChange={(e) => setFormData({ ...formData, hardwareExperience: e.target.value })}
              placeholder="e.g., Arduino, Raspberry Pi, sensors, etc."
              rows={3}
            />
          </div>

          <div className={styles.formGroup}>
            <label>Areas of Interest (select all that apply)</label>
            <div className={styles.interestGrid}>
              {interestOptions.map((interest) => (
                <button
                  key={interest}
                  type="button"
                  className={`${styles.interestButton} ${
                    formData.interests.includes(interest) ? styles.selected : ''
                  }`}
                  onClick={() => handleInterestToggle(interest)}
                >
                  {interest}
                </button>
              ))}
            </div>
          </div>

          <button type="submit" className={styles.submitButton} disabled={isLoading}>
            {isLoading ? 'Creating Account...' : 'Sign Up'}
          </button>
        </form>

        <p className={styles.switchAuth}>
          Already have an account? <a href="/signin">Sign In</a>
        </p>
      </div>
    </div>
  );
}

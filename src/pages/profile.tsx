import React from 'react';
import Layout from '@theme/Layout';
import styles from './profile.module.css';

export default function ProfilePage() {
  // TODO: Get user data from Better-Auth session
  const user = {
    name: 'John Doe',
    email: 'john@example.com',
    background: 'intermediate',
    interests: ['ROS 2', 'Computer Vision', 'Humanoid Robotics'],
    softwareExperience: 'Python, C++, JavaScript',
    hardwareExperience: 'Arduino, Raspberry Pi',
  };

  return (
    <Layout title="Profile" description="Your profile">
      <div className={styles.profileContainer}>
        <div className={styles.profileCard}>
          <div className={styles.header}>
            <div className={styles.avatar}>
              {user.name.charAt(0).toUpperCase()}
            </div>
            <div className={styles.userInfo}>
              <h1>{user.name}</h1>
              <p>{user.email}</p>
            </div>
          </div>

          <div className={styles.section}>
            <h2>Experience Level</h2>
            <div className={styles.badge}>
              {user.background.charAt(0).toUpperCase() + user.background.slice(1)}
            </div>
          </div>

          <div className={styles.section}>
            <h2>Areas of Interest</h2>
            <div className={styles.interestList}>
              {user.interests.map((interest) => (
                <span key={interest} className={styles.interestTag}>
                  {interest}
                </span>
              ))}
            </div>
          </div>

          <div className={styles.section}>
            <h2>Software Experience</h2>
            <p className={styles.text}>{user.softwareExperience}</p>
          </div>

          <div className={styles.section}>
            <h2>Hardware Experience</h2>
            <p className={styles.text}>{user.hardwareExperience}</p>
          </div>

          <div className={styles.actions}>
            <button className={styles.editButton}>Edit Profile</button>
            <button className={styles.logoutButton}>Sign Out</button>
          </div>
        </div>
      </div>
    </Layout>
  );
}

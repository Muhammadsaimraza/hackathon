import React, { useState } from 'react';
import styles from './BetterAuth.module.css';

export default function BetterAuth() {
  const [user, setUser] = useState(null);

  const handleLogin = () => {
    // Simulate login
    setUser({ name: 'Student' });
  };

  const handleLogout = () => {
    setUser(null);
  };

  return (
    <div className="navbar__item">
      {user ? (
        <div style={{ display: 'flex', alignItems: 'center', gap: '10px' }}>
          <span>Hello, {user.name}</span>
          <button
            className="button button--secondary button--sm"
            onClick={handleLogout}
          >
            Log Out
          </button>
        </div>
      ) : (
        <button
          className="button button--primary button--sm"
          onClick={handleLogin}
        >
          Log In
        </button>
      )}
    </div>
  );
}

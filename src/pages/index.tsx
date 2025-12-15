
import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

function HomepageHeader() {
  return (
    <header className={styles.heroBanner}>
      <div className="container">
        <h1 className={styles.heroTitle}>ROBOLEARN PLATFORM</h1>
        <p className={styles.heroSubtitle}>Build robots that understand the physical world.</p>
        <p className={styles.heroDescription}>Master Physical AI from browser to production. ROS 2, Isaac Sim, and Vision-Language-Action models. Free forever.</p>
        {/* You can add a button here if you want */}
      </div>
    </header>
  );
}

function JourneySection() {
    return (
        <section className={styles.journeySection}>
            <div className="container">
                <h2>YOUR JOURNEY</h2>
                <h3>From Zero to Building Robots That Think</h3>
                <p>Each step brings you closer to creating machines that move, sense, and understand.</p>
            </div>
        </section>
    );
}

const modules = [
  {
    title: 'Module 1: Robot Middleware',
    subtitle: 'ROS 2 Fundamentals',
    icon: '🔌',
    description: 'Master the robotic nervous system with ROS 2 middleware for robot control.',
    details: ['Nodes, Topics, and Services', 'Python Agents with rclpy', 'URDF for Humanoids'],
    duration: 'Weeks 1-5',
  },
  {
    title: 'Module 2: Simulation',
    subtitle: 'Digital Twins',
    icon: '🤖',
    description: 'Build physics simulations and high-fidelity environments.',
    details: ['Gazebo Physics Simulation', 'Unity Visualization', 'Sensor Simulation (LiDAR, IMU)'],
    duration: 'Weeks 6-7',
  },
  {
    title: 'Module 3: AI-Powered',
    subtitle: 'NVIDIA Isaac',
    icon: '🧠',
    description: 'Advanced perception, navigation, and sim-to-real transfer.',
    details: ['Isaac Sim & Synthetic Data', 'VSLAM & Navigation', 'Reinforcement Learning'],
    duration: 'Weeks 8-10',
  },
  {
    title: 'Module 4: Capstone',
    subtitle: 'Vision-Language-Action',
    icon: '🏆',
    description: 'Convergence of LLMs and Robotics for conversational control.',
    details: ['Voice-to-Action (Whisper)', 'LLM Cognitive Planning', 'Autonomous Humanoid Capstone'],
    duration: 'Weeks 11-13',
  },
];

function ModulesSection() {
    return (
        <section className={styles.modulesSection}>
            <div className="container">
                <div className={styles.modulesGrid}>
                    {modules.map((module, idx) => (
                        <div key={idx} className={styles.moduleCard}>
                            <div className={styles.moduleIcon}>{module.icon}</div>
                            <h4>{module.title}</h4>
                            <h5>{module.subtitle}</h5>
                            <p>{module.description}</p>
                            <ul>
                                {module.details.map((detail, i) => (
                                    <li key={i}>▸ {detail}</li>
                                ))}
                            </ul>
                            <span>{module.duration}</span>
                        </div>
                    ))}
                </div>
            </div>
        </section>
    );
}


function WhyMattersSection() {
    return (
        <section className={styles.whyMattersSection}>
            <div className="container">
                <h2>Why This Matters</h2>
                <h3>Build Machines That Free Up Your Time</h3>
                <p>Robots that handle physical tasks while you focus on what matters most.</p>
            </div>
        </section>
    );
}

const features = [
    {
        title: 'Embodied Intelligence',
        description: 'AI that operates in physical space, not just digital environments. Robots that understand physics and interact with the real world.'
    },
    {
        title: 'Human-Centered Design',
        description: 'Humanoid robots navigate our world without modification. They use human tools, interfaces, and learn from demonstrations.'
    },
    {
        title: 'Production-Ready Skills',
        description: 'ROS 2, Gazebo, NVIDIA Isaac, and VLA models. The complete stack for modern robotics development.'
    },
    {
        title: 'Conversational Robotics',
        description: "Natural language commands translated to robot actions. 'Clean the room' becomes a sequence of coordinated movements."
    },
    {
        title: 'Sim-to-Real Transfer',
        description: 'Train in simulation, deploy to reality. Photorealistic environments and domain randomization bridge the gap.'
    },
    {
        title: 'Interactive Learning',
        description: 'RAG-powered chat, personalized content, and hands-on exercises. Learn by doing, not just reading.'
    }
];

function FeaturesSection() {
    return (
        <section className={styles.featuresSection}>
            <div className="container">
                <div className={styles.featuresGrid}>
                    {features.map((feature, idx) => (
                        <div key={idx} className={styles.featureCard}>
                            <h4>{feature.title}</h4>
                            <p>{feature.description}</p>
                        </div>
                    ))}
                </div>
            </div>
        </section>
    );
}

function StartSection() {
    return (
        <section className={styles.startSection}>
            <div className="container">
                <h2>Start Where You Are</h2>
                <h3>No Expensive Hardware Required</h3>
                <p>Begin building today with just your browser. Scale up when you're ready.</p>
            </div>
        </section>
    );
}

const hardwareOptions = [
    {
        tier: 1,
        title: 'Workstation',
        subtitle: 'Full Local Setup',
        description: 'Best Experience',
        details: 'RTX 4070 Ti+, 64GB RAM, Ubuntu 22.04. Run Isaac Sim locally with full performance.',
        cost: 'Cost: ~$2,500+ hardware',
    },
    {
        tier: 2,
        title: 'Cloud + Edge',
        subtitle: 'Hybrid Approach',
        description: 'Flexible',
        details: 'AWS/Azure GPU instances for simulation. Jetson kit for physical deployment.',
        cost: 'Cost: ~$200/quarter cloud + $700 Jetson',
    },
    {
        tier: 3,
        title: 'Simulation Only',
        subtitle: 'Learning Focus',
        description: 'Lowest Cost',
        details: 'Cloud-based simulation without physical hardware. Complete the theory and simulation modules.',
        cost: 'Cost: Cloud compute only',
    }
];

function HardwareSection() {
    return (
        <section className={styles.hardwareSection}>
            <div className="container">
                <div className={styles.hardwareGrid}>
                    {hardwareOptions.map((option, idx) => (
                        <div key={idx} className={styles.hardwareCard}>
                            <div className={styles.hardwareTier}>{option.tier}</div>
                            <h4>{option.title}</h4>
                            <h5>{option.subtitle}</h5>
                            <p>{option.description}</p>
                            <p>{option.details}</p>
                            <span>{option.cost}</span>
                        </div>
                    ))}
                </div>
            </div>
        </section>
    );
}

function CtaSection() {
    return (
        <section className={styles.ctaSection}>
            <div className="container">
                <h2>Ready to Begin?</h2>
                <h3>Future is Physical AI. Robots That Think, Move, and Collaborate.</h3>
                <p>Join the transition from AI confined to screens to AI that shapes the physical world alongside us.</p>
                <p>Start Your Physical AI Journey. From ROS 2 basics to autonomous humanoids with voice control.</p>
                <Link
                    className="button button--primary button--lg"
                    to="/docs/Introduction/">
                    GET STARTED FREE
                </Link>
            </div>
        </section>
    );
}


export default function Home(): React.ReactElement {
  return (
    <Layout
      title="RoboLearn"
      description="Master Physical AI from browser to production.">
      <HomepageHeader />
      <main>
        <JourneySection />
        <ModulesSection />
        <WhyMattersSection />
        <FeaturesSection />
        <StartSection />
        <HardwareSection />
        <CtaSection />
      </main>
    </Layout>
  );
}

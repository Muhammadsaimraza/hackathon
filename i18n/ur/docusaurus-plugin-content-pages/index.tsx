import React from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import styles from './index.module.css';

function HomepageHeader() {
  return (
    <header className={styles.heroBanner}>
      <div className="container">
        <h1 className={styles.heroTitle}>روبولرن پلیٹ فارم</h1>
        <p className={styles.heroSubtitle}>ایسے روبوٹ بنائیں جو طبعی دنیا کو سمجھیں۔</p>
        <p className={styles.heroDescription}>براؤزر سے پروڈکشن تک فزیکل اے آئی میں مہارت حاصل کریں۔ ROS 2، آئزک سم، اور ویژن-لینگویج-ایکشن ماڈلز۔ ہمیشہ کے لیے مفت۔</p>
        {/* You can add a button here if you want */}
      </div>
    </header>
  );
}

function JourneySection() {
    return (
        <section className={styles.journeySection}>
            <div className="container">
                <h2>آپ کا سفر</h2>
                <h3>صفر سے سوچنے والے روبوٹ بنانے تک</h3>
                <p>ہر قدم آپ کو ایسی مشینیں بنانے کے قریب لاتا ہے جو حرکت کرتی ہیں، محسوس کرتی ہیں اور سمجھتی ہیں۔</p>
            </div>
        </section>
    );
}

const modules = [
  {
    title: 'ماڈیول 1: روبوٹ مڈل ویئر',
    subtitle: 'ROS 2 کے بنیادی اصول',
    icon: '🔌',
    description: 'روبوٹ کنٹرول کے لیے ROS 2 مڈل ویئر کے ساتھ روبوٹک نروس سسٹم میں مہارت حاصل کریں۔',
    details: ['نوڈس، ٹاپکس، اور سروسز', 'rclpy کے ساتھ پائتھن ایجنٹس', 'ہیومنائڈز کے لیے URDF'],
    duration: 'ہفتے 1-5',
  },
  {
    title: 'ماڈیول 2: سیمولیشن',
    subtitle: 'ڈیجیٹل ٹوئنز',
    icon: '🤖',
    description: 'طبیعیات کی سمیولیشن اور ہائی فیڈیلیٹی ماحول بنائیں۔',
    details: ['گیزیبو فزکس سمیولیشن', 'یونٹی ویژولائزیشن', 'سینسر سمیولیشن (LiDAR، IMU)'],
    duration: 'ہفتے 6-7',
  },
  {
    title: 'ماڈیول 3: اے آئی پاورڈ',
    subtitle: 'این ویڈیا آئزک',
    icon: '🧠',
    description: 'جدید پرسیپشن، نیویگیشن، اور سم-ٹو-ریئل ٹرانسفر۔',
    details: ['آئزک سم اور مصنوعی ڈیٹا', 'VSLAM اور نیویگیشن', 'ری انفورسمنٹ لرننگ'],
    duration: 'ہفتے 8-10',
  },
  {
    title: 'ماڈیول 4: کیپسٹون',
    subtitle: 'ویژن-لینگویج-ایکشن',
    icon: '🏆',
    description: 'بات چیت کے کنٹرول کے لیے LLMs اور روبوٹکس کا سنگم۔',
    details: ['وائس ٹو ایکشن (Whisper)', 'LLM علمی منصوبہ بندی', 'خود مختار ہیومنائڈ کیپسٹون'],
    duration: 'ہفتے 11-13',
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
                <h2>یہ کیوں اہم ہے</h2>
                <h3>ایسی مشینیں بنائیں جو آپ کا وقت بچائیں</h3>
                <p>روبوٹ جو جسمانی کام سنبھالتے ہیں جبکہ آپ ان چیزوں پر توجہ مرکوز کرتے ہیں جو سب سے زیادہ اہم ہیں۔</p>
            </div>
        </section>
    );
}

const features = [
    {
        title: 'مجسم ذہانت',
        description: 'وہ اے آئی جو صرف ڈیجیٹل ماحول میں نہیں بلکہ طبعی جگہ میں کام کرتی ہے۔ وہ روبوٹ جو طبیعیات کو سمجھتے ہیں اور حقیقی دنیا کے ساتھ تعامل کرتے ہیں۔'
    },
    {
        title: 'انسان مرکز ڈیزائن',
        description: 'ہیومنائڈ روبوٹ بغیر کسی ترمیم کے ہماری دنیا میں نیویگیٹ کرتے ہیں۔ وہ انسانی اوزار، انٹرفیس استعمال کرتے ہیں، اور مظاہروں سے سیکھتے ہیں۔'
    },
    {
        title: 'پروڈکشن کے لیے تیار مہارتیں',
        description: 'ROS 2، گیزیبو، این ویڈیا آئزک، اور VLA ماڈلز۔ جدید روبوٹکس ڈویلپمنٹ کے لیے مکمل اسٹیک۔'
    },
    {
        title: 'بات چیت والی روبوٹکس',
        description: "قدرتی زبان کے احکامات روبوٹ کے اعمال میں ترجمہ ہوتے ہیں۔ 'کمرہ صاف کرو' مربوط حرکات کا ایک سلسلہ بن جاتا ہے۔"
    },
    {
        title: 'سم-ٹو-ریئل ٹرانسفر',
        description: 'سمیولیشن میں تربیت دیں، حقیقت میں تعینات کریں۔ فوٹو ریئلسٹک ماحول اور ڈومین رینڈمائزیشن فرق کو ختم کرتے ہیں۔'
    },
    {
        title: 'انٹرایکٹو لرننگ',
        description: 'RAG پاورڈ چیٹ، ذاتی نوعیت کا مواد، اور ہینڈز آن مشقیں۔ صرف پڑھنے سے نہیں، کرنے سے سیکھیں۔'
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
                <h2>وہیں سے شروع کریں جہاں آپ ہیں</h2>
                <h3>کسی مہنگے ہارڈ ویئر کی ضرورت نہیں</h3>
                <p>آج ہی صرف اپنے براؤزر سے تعمیر شروع کریں۔ جب آپ تیار ہوں تو پیمانہ بڑھائیں۔</p>
            </div>
        </section>
    );
}

const hardwareOptions = [
    {
        tier: 1,
        title: 'ورک سٹیشن',
        subtitle: 'مکمل مقامی سیٹ اپ',
        description: 'بہترین تجربہ',
        details: 'RTX 4070 Ti+, 64GB RAM, Ubuntu 22.04. مکمل کارکردگی کے ساتھ مقامی طور پر آئزک سم چلائیں۔',
        cost: 'قیمت: ~$2,500+ ہارڈ ویئر',
    },
    {
        tier: 2,
        title: 'کلاؤڈ + ایج',
        subtitle: 'ہائبرڈ نقطہ نظر',
        description: 'لچکدار',
        details: 'سمیولیشن کے لیے AWS/Azure GPU مثالیں۔ جسمانی تعیناتی کے لیے جیٹسن کٹ۔',
        cost: 'قیمت: ~$200/سہ ماہی کلاؤڈ + $700 جیٹسن',
    },
    {
        tier: 3,
        title: 'صرف سیمولیشن',
        subtitle: 'سیکھنے کی توجہ',
        description: 'کم ترین قیمت',
        details: 'جسمانی ہارڈ ویئر کے بغیر کلاؤڈ بیسڈ سمیولیشن۔ تھیوری اور سمیولیشن کے ماڈیولز مکمل کریں۔',
        cost: 'قیمت: صرف کلاؤڈ کمپیوٹ',
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
                <h2>شروع کرنے کے لیے تیار ہیں؟</h2>
                <h3>مستقبل فزیکل اے آئی ہے۔ روبوٹ جو سوچتے، حرکت کرتے اور تعاون کرتے ہیں۔</h3>
                <p>صرف اسکرینوں تک محدود اے آئی سے اس اے آئی کی طرف منتقلی میں شامل ہوں جو ہمارے ساتھ طبعی دنیا کو تشکیل دیتی ہے۔</p>
                <p>اپنا فزیکل اے آئی کا سفر شروع کریں۔ ROS 2 کی بنیادی باتوں سے لے کر وائس کنٹرول کے ساتھ خود مختار ہیومنائڈز تک۔</p>
                <Link
                    className="button button--primary button--lg"
                    to="/docs/Introduction/">
                    مفت شروع کریں
                </Link>
            </div>
        </section>
    );
}


export default function Home(): React.ReactElement {
  return (
    <Layout
      title="روبولرن"
      description="براؤزر سے پروڈکشن تک فزیکل اے آئی میں مہارت حاصل کریں۔">
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
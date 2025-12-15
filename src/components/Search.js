import React, {useState} from 'react';
import styles from './Search.module.css';

export default function Search() {
  const [query, setQuery] = useState('');

  const handleInputChange = (event) => {
    setQuery(event.target.value);
  };

  const handleSearch = (event) => {
    event.preventDefault();
    // For now, let's just log to the console.
    // In a real implementation, you would perform a search action here.
    // For example, redirecting to a search results page
    // or calling a search API.
    console.log(`Searching for used books with query: ${query}`);
    alert(`Searching for used books with query: "${query}"`);
  };

  return (
    <form onSubmit={handleSearch} className={styles.searchContainer}>
      <input
        type="search"
        value={query}
        onChange={handleInputChange}
        placeholder="Search for used books..."
        className={styles.searchInput}
      />
      <button type="submit" className={styles.searchButton}>Search</button>
    </form>
  );
}
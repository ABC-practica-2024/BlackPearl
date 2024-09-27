import { Link } from "react-router-dom";
import styles from "./Homepage.module.css";

export default function Homepage() {
  return (
    <main className={styles.homepage}>
      <section>
        <h1>
          ABC Practica
          <br />
          Barcuta
        </h1>
        <h2>
          Ceva descriere fancy
        </h2>
        <Link to="/app" className="cta">
          Look at the boat!
        </Link>
      </section>
    </main>
  );
}

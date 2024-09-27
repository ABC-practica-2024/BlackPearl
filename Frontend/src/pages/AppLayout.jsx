import Map from "../components/Map";

import styles from "./AppLayout.module.css";

function AppLayout() {
  return (
    <div className={styles.app}>
      <Map />
    </div>
  );
}

export default AppLayout;

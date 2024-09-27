import { useNavigate, useSearchParams } from "react-router-dom";
import {
  MapContainer,
  TileLayer,
  Marker,
  Popup,
  useMap,
  useMapEvents,
} from "react-leaflet";

import styles from "./Map.module.css";
import { useEffect, useState } from "react";
import { useGeolocation } from "../hooks/useGeolocation";
import { useUrlPosition } from "../hooks/useUrlPosition";
import Button from "./Button";

function Map() {
  const [mapPosition, setMapPosition] = useState([45.5, 24.5]);
  const {
    isLoading: isLoadingPosition,
    position: geolocationPosition,
    getPosition,
  } = useGeolocation();
  const [mapLat, mapLng] = useUrlPosition();

  useEffect(
    function () {
      if (mapLat && mapLng) setMapPosition([mapLat, mapLng]);
    },
    [mapLat, mapLng]
  );

  useEffect(
    function () {
      if (geolocationPosition)
        setMapPosition([geolocationPosition.lat, geolocationPosition.lng]);
    },
    [geolocationPosition]
  );
  return (
  <div className={styles.mapWrapper}>
    <div className={styles.mapContainer}>
      {/* iti ia pozitia locala , doar ca o ia pe a dispozitivului, aici va fi implementata pozitia primita din MQTT de la barcuta */}
      {!geolocationPosition && (
        <button className={styles.mapButton} onClick={getPosition}>
          {isLoadingPosition ? "Loading..." : "Use your position"}
        </button>
      )}

      <MapContainer
        center={mapPosition}
        zoom={6}
        scrollWheelZoom={true}
        className={styles.map}
      >
        <TileLayer
          attribution='&copy; <a href="https://maps.google.com">Google Maps</a>'
          url="http://{s}.google.com/vt/lyrs=m&x={x}&y={y}&z={z}"
          subdomains={['mt0', 'mt1', 'mt2', 'mt3']}
        />
        <ChangeCenter position={mapPosition} />
        {/* <DetectClick /> */}
      </MapContainer>
    </div>
  </div>
);
}

function ChangeCenter({ position }) {
  const map = useMap();
  map.setView(position);
  return null;
}

// function DetectClick() {
//   const navigate = useNavigate();

//   useMapEvents({
        //aici va fi facut form-ul sau ceva de genul prin care se va adauga un marker la care sa putem da go to coordinates
//     click: (e) => navigate(`form?lat=${e.latlng.lat}&lng=${e.latlng.lng}`),
//   });
// }

export default Map;

import { lazy } from "react";
import { BrowserRouter, Routes, Route } from "react-router-dom";

const Homepage = lazy(() => import("./pages/Homepage"));
const AppLayout = lazy(() => import("./pages/AppLayout"));
const PageNotFound = lazy(() => import("./pages/PageNotFound"));

function App() {
  return (
        <BrowserRouter>
            <Routes>
              <Route index element={<Homepage />} />
              <Route
                path="app"
                element={
                  <AppLayout />
                }
              >
              </Route>
              <Route path="*" element={<PageNotFound />} />
            </Routes>
        </BrowserRouter>
  );
}

export default App;

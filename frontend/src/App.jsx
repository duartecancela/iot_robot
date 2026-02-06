import Header from './components/Header'
import Footer from './components/Footer'
import TelemetryState from './components/TelemetryState'
import DriveJoystick from './components/DriveJoystick'



function App() {
  return (
    <div className="min-h-screen flex flex-col font-sans">
      <Header />

      <main className="flex-1 max-w-5xl mx-auto p-6">
        <TelemetryState />
        <DriveJoystick />
      </main>
      

      <Footer />
    </div>
  )
}

export default App

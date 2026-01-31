import Header from './components/Header'
import Footer from './components/Footer'

function App() {
  return (
    <div className="min-h-screen flex flex-col font-sans">
      <Header />

      <main className="flex-1 max-w-5xl mx-auto p-6">
        <h1 className="text-3xl font-bold text-blue-600">
          Hello World 👋
        </h1>

        <p className="mt-4 text-gray-600">
          Tailwind CSS está ativo e a funcionar.
        </p>

        <div className="mt-6 p-4 border rounded-lg bg-gray-50">
          <span className="text-sm text-gray-500">Status:</span>
          <span className="ml-2 font-mono text-green-600">OK</span>
        </div>
      </main>
      

      <Footer />
    </div>
  )
}

export default App

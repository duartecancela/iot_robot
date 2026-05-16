import { useEffect, useState } from 'react'

const BACKEND_URL = import.meta.env.VITE_BACKEND_URL

export default function LoggingControl() {
  const [logging, setLogging] = useState({
    enabled: false,
    startedAt: null,
  })

  async function loadStatus() {
    try {
      const res = await fetch(`${BACKEND_URL}/logging/status`, {
        cache: 'no-store',
      })

      const json = await res.json()

      if (json?.logging) {
        setLogging(json.logging)
      }
    } catch (err) {
      console.log('Logging status error:', err.message)
    }
  }

  async function toggleLogging(enabled) {
    try {
      await fetch(`${BACKEND_URL}/logging`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ enabled }),
      })

      loadStatus()
    } catch (err) {
      console.log('Logging toggle error:', err.message)
    }
  }

  useEffect(() => {
    loadStatus()

    const interval = setInterval(loadStatus, 3000)

    return () => clearInterval(interval)
  }, [])

  return (
    <section className="mt-4">
      <div className="mb-2 flex items-center justify-between gap-2">
        <div className="flex items-center gap-2">
          <h2 className="text-sm font-semibold text-stone-900">
            MongoDB Logging
          </h2>

          <span
            className={`rounded px-2 py-0.5 text-xs ${
              logging.enabled
                ? 'bg-green-100 text-green-700'
                : 'bg-stone-100 text-stone-700'
            }`}
          >
            {logging.enabled ? 'ACTIVE' : 'OFF'}
          </span>
        </div>
      </div>

      <div className="rounded border border-stone-200 bg-stone-50/40 p-2">
        <div className="flex gap-2">
          <button
            onClick={() => toggleLogging(true)}
            disabled={logging.enabled}
            className="rounded border border-stone-300 bg-stone-50 px-2 py-1 text-xs text-stone-800 hover:bg-stone-100 disabled:opacity-50"
          >
            Enable
          </button>

          <button
            onClick={() => toggleLogging(false)}
            disabled={!logging.enabled}
            className="rounded border border-stone-300 bg-stone-50 px-2 py-1 text-xs text-stone-800 hover:bg-stone-100 disabled:opacity-50"
          >
            Disable
          </button>
        </div>

        <div className="mt-2 text-xs text-stone-600">
          {logging.startedAt ? (
            <>
              Started:{' '}
              {new Date(logging.startedAt).toLocaleTimeString('pt-PT', {
                hour12: false,
              })}
            </>
          ) : (
            <>Logging inactive</>
          )}
        </div>
      </div>
    </section>
  )
}
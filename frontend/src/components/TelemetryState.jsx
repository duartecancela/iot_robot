import { useEffect, useRef, useState } from 'react'

const BACKEND_URL = import.meta.env.VITE_BACKEND_URL

export default function TelemetryState() {
  const [data, setData] = useState(null)
  const [error, setError] = useState(null)
  const [loading, setLoading] = useState(true)

  // Prevent overlapping requests (important for VPN/slow links)
  const inFlightRef = useRef(false)

  async function load({ showLoading = false } = {}) {
    // Lock: if a request is already running, skip this tick
    if (inFlightRef.current) return
    inFlightRef.current = true

    if (showLoading) setLoading(true)

    try {
      setError(null)
      const res = await fetch(`${BACKEND_URL}/telemetry/state`)
      if (!res.ok) throw new Error(`HTTP ${res.status}`)
      const json = await res.json()
      setData(json)
    } catch (e) {
      setError(e?.message || 'Failed to load telemetry')
    } finally {
      inFlightRef.current = false
      setLoading(false)
    }
  }

  useEffect(() => {
    load({ showLoading: true }) // initial load with loading indicator

    const interval = setInterval(() => {
      // Poll silently (no "Loading..." flicker)
      load({ showLoading: false })
    }, 1000)

    return () => clearInterval(interval)
  }, [])

  return (
    <section className="mt-6">
      <div className="flex items-center justify-between">
        <h2 className="text-lg font-semibold">Telemetry State</h2>

        <button
          onClick={() => load({ showLoading: true })}
          className="px-3 py-2 text-sm border rounded-md hover:bg-gray-50"
        >
          Refresh
        </button>
      </div>

      <div className="mt-3 border rounded-lg p-4 bg-white">
        {loading && <p className="text-sm text-gray-500">Loading…</p>}

        {error && (
          <p className="text-sm text-red-600">
            Error: {error}
          </p>
        )}

        {!loading && !error && data && (
          <div className="space-y-4">
            {/* FAST */}
            <div className="border rounded-lg p-4">
              <div className="flex items-center justify-between">
                <h3 className="font-semibold">Fast Telemetry</h3>
                <span className="text-xs text-gray-500 font-mono">{data.fast?.topic}</span>
              </div>

              {!data.fast?.json ? (
                <p className="mt-2 text-sm text-gray-500">No fast telemetry yet.</p>
              ) : (
                <div className="mt-3 grid gap-3 sm:grid-cols-2">
                  <div className="border rounded-lg p-3 bg-gray-50">
                    <div className="text-xs text-gray-500">ToF (mm)</div>
                    <div className="mt-1 text-sm">
                      FL: <span className="font-mono">{data.fast.json.tof?.fl}</span> &nbsp;|&nbsp;
                      FR: <span className="font-mono">{data.fast.json.tof?.fr}</span>
                    </div>
                  </div>

                  <div className="border rounded-lg p-3 bg-gray-50">
                    <div className="text-xs text-gray-500">IMU (deg)</div>
                    <div className="mt-1 text-sm">
                      Pitch: <span className="font-mono">{data.fast.json.imu?.pitch}</span> &nbsp;|&nbsp;
                      Roll: <span className="font-mono">{data.fast.json.imu?.roll}</span>
                    </div>
                  </div>
                </div>
              )}
            </div>

            {/* SLOW */}
            <div className="border rounded-lg p-4">
              <div className="flex items-center justify-between">
                <h3 className="font-semibold">Slow Telemetry</h3>
                <span className="text-xs text-gray-500 font-mono">{data.slow?.topic}</span>
              </div>

              {!data.slow?.json ? (
                <p className="mt-2 text-sm text-gray-500">No slow telemetry yet.</p>
              ) : (
                <div className="mt-3 grid gap-3 sm:grid-cols-3">
                  <div className="border rounded-lg p-3 bg-gray-50">
                    <div className="text-xs text-gray-500">Temperature (°C)</div>
                    <div className="mt-1 text-sm font-mono">{data.slow.json.bme?.t}</div>
                  </div>

                  <div className="border rounded-lg p-3 bg-gray-50">
                    <div className="text-xs text-gray-500">Humidity (%)</div>
                    <div className="mt-1 text-sm font-mono">{data.slow.json.bme?.h}</div>
                  </div>

                  <div className="border rounded-lg p-3 bg-gray-50">
                    <div className="text-xs text-gray-500">Pressure (hPa)</div>
                    <div className="mt-1 text-sm font-mono">{data.slow.json.bme?.p}</div>
                  </div>
                </div>
              )}
            </div>
          </div>
        )}
      </div>
    </section>
  )
}

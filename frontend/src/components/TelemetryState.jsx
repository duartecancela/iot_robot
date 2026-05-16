import { useEffect, useRef, useState } from 'react'
import {
  ArrowsRightLeftIcon,
  ArrowPathIcon,
  SunIcon,
  CloudIcon,
  AdjustmentsHorizontalIcon,
  TruckIcon,
  CheckCircleIcon,
  XCircleIcon,
} from '@heroicons/react/24/outline'

const BACKEND_URL =
  import.meta.env.VITE_BACKEND_URL || 'http://iotrobot.local:3001'

function Cell({ label, value, Icon, variant = 'base' }) {
  const variants = {
    base: 'bg-white border-stone-200',
    tof: 'bg-amber-50/60 border-amber-200/60',
    imu: 'bg-slate-50/70 border-slate-200/70',
    bme: 'bg-emerald-50/55 border-emerald-200/60',
    drive: 'bg-rose-50/55 border-rose-200/60',
    tracking: 'bg-indigo-50/60 border-indigo-200/60',
  }

  return (
    <div className={`rounded border px-2 py-1.5 ${variants[variant]}`}>
      <div className="flex items-center gap-1 text-[10px] uppercase tracking-wide text-stone-600">
        {Icon && <Icon className="h-3.5 w-3.5 opacity-70" />}
        <span>{label}</span>
      </div>
      <div className="mt-0.5 font-mono text-xs text-stone-900">
        {value ?? '—'}
      </div>
    </div>
  )
}

export default function TelemetryState() {
  const [data, setData] = useState(null)
  const [error, setError] = useState(null)
  const [loading, setLoading] = useState(true)

  const inFlightRef = useRef(false)

  async function load({ showLoading = false } = {}) {
    if (inFlightRef.current) return
    inFlightRef.current = true

    if (showLoading) setLoading(true)

    try {
      setError(null)
      const res = await fetch(`${BACKEND_URL}/telemetry/state`, { cache: 'no-store' })
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
    load({ showLoading: true })
    const interval = setInterval(() => load({ showLoading: false }), 1000)
    return () => clearInterval(interval)
  }, [])

  const fast = data?.fast?.json
  const slow = data?.slow?.json
  const driveState = data?.drive?.state?.json
  const driveAck = data?.drive?.ack?.json
  const tracking = data?.tracking

  // 🔥 FIXED toggle
  async function toggleTracking() {
    if (!tracking) return

    const newValue = !tracking.tracking_allowed

    await fetch(`${BACKEND_URL}/tracking`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ tracking_allowed: newValue }),
    })

    // refresh imediato (UX melhor)
    load()
  }

  const ts = data?.ts ?? Date.now()
  const time24 = new Date(ts).toLocaleTimeString('pt-PT', {
    hour12: false,
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
  })

  return (
    <section className="mt-4">
      <div className="mb-2 flex items-center justify-between gap-2">
        <div className="flex items-center gap-2">
          <h2 className="text-sm font-semibold text-stone-900">Telemetry</h2>

          {loading ? (
            <span className="rounded bg-stone-100 px-2 py-0.5 text-xs text-stone-700">
              Loading…
            </span>
          ) : error ? (
            <span className="rounded bg-red-50 px-2 py-0.5 text-xs text-red-700">
              Error
            </span>
          ) : (
            <span className="rounded bg-stone-100 px-2 py-0.5 text-xs text-stone-700">
              {time24}
            </span>
          )}
        </div>

        <button
          onClick={() => load({ showLoading: true })}
          className="rounded border border-stone-300 bg-stone-50 px-2 py-1 text-xs text-stone-800 hover:bg-stone-100"
        >
          Refresh
        </button>
      </div>

      <div className="rounded border border-stone-200 bg-stone-50/40 p-2">
        {error && (
          <div className="mb-2 text-xs text-red-700">
            Error: {error}
          </div>
        )}

        <div className="grid grid-cols-2 gap-2 sm:grid-cols-4">

          <Cell label="ToF FL" value={fast?.tof?.fl} Icon={ArrowsRightLeftIcon} variant="tof" />
          <Cell label="ToF FR" value={fast?.tof?.fr} Icon={ArrowsRightLeftIcon} variant="tof" />

          <Cell label="Pitch" value={fast?.imu?.pitch} Icon={ArrowPathIcon} variant="imu" />
          <Cell label="Roll" value={fast?.imu?.roll} Icon={ArrowPathIcon} variant="imu" />

          <Cell label="Temp °C" value={slow?.bme?.t} Icon={SunIcon} variant="bme" />
          <Cell label="Hum %" value={slow?.bme?.h} Icon={CloudIcon} variant="bme" />
          <Cell label="Press hPa" value={slow?.bme?.p} Icon={AdjustmentsHorizontalIcon} variant="bme" />

          <Cell label="Drive" value={driveState ? `${driveState.left},${driveState.right}` : null} Icon={TruckIcon} variant="drive" />
          <Cell label="Ack" value={driveAck ? (driveAck.ok ? 'OK' : 'ERR') : null} Icon={driveAck?.ok ? CheckCircleIcon : XCircleIcon} variant="drive" />

          {/* TRACKING */}
          <Cell label="Moving" value={tracking?.robot_is_moving ? 'YES' : 'NO'} variant="tracking" />
          <Cell label="Tracking Active" value={tracking?.tracking_active ? 'ON' : 'OFF'} variant="tracking" />
          <Cell label="Tracking Allowed" value={tracking?.tracking_allowed ? 'YES' : 'NO'} variant="tracking" />

        </div>

        <div className="mt-3 flex justify-center">
          <button
            onClick={toggleTracking}
            disabled={!tracking}
            className={`px-4 py-2 text-sm rounded ${
              tracking?.tracking_allowed
                ? 'bg-red-500 text-white'
                : 'bg-green-500 text-white'
            }`}
          >
            {tracking?.tracking_allowed ? 'Disable Tracking' : 'Enable Tracking'}
          </button>
        </div>

      </div>
    </section>
  )
}
import { useMemo, useState } from 'react'

const DEFAULT_STREAM_URL = 'http://iotrobot.local:8080/stream.mjpg'

export default function VideoStream() {
  const [error, setError] = useState(false)

  const streamUrl = useMemo(() => {
    return import.meta.env.VITE_STREAM_URL || DEFAULT_STREAM_URL
  }, [])

  return (
    <section className="mt-4">
      <div className="mb-2 flex items-center justify-between gap-2">
        <div className="flex items-center gap-2">
          <h2 className="text-sm font-semibold text-stone-900">Live Stream</h2>
          <span className="rounded bg-stone-100 px-2 py-0.5 text-xs text-stone-700">
            MJPEG
          </span>
        </div>

        <a
          href={streamUrl}
          target="_blank"
          rel="noreferrer"
          className="text-xs text-blue-600 underline"
        >
          Open stream
        </a>
      </div>

      <div className="rounded border border-stone-200 bg-stone-50/40 p-2">
        <div className="overflow-hidden rounded border border-stone-200 bg-black">
          <div className="flex justify-center">
            <iframe
              src={streamUrl}
              title="Robot live stream"
              className="h-[240px] w-[320px] rounded border-0 bg-black"
            />
          </div>
        </div>

        {error && (
          <div className="mt-2 rounded border border-red-200 bg-red-50 p-2 text-xs text-red-700">
            Stream could not be loaded inside the frontend.
          </div>
        )}

        <div className="mt-2 text-xs text-stone-600 break-all">
          Source: {streamUrl}
        </div>
      </div>
    </section>
  )
}
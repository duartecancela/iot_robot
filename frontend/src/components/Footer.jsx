export default function Footer() {
  return (
    <footer className="border-t mt-8">
      <div className="max-w-5xl mx-auto px-6 py-4">
        <small className="text-gray-500">
          © {new Date().getFullYear()} Duarte Cancela - IPBeja • IoT Robot
        </small>
      </div>
    </footer>
  )
}

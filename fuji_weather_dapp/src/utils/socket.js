import { OFFERS_API } from '../config'

let socket = null
const setSocket = (cb) => {
  if (socket === null) {
    socket = io(OFFERS_API)
    socket.on('connect', () => {
      console.log('connect')
      cb(socket)
    })
    return
  }
  cb(socket)
}

export default setSocket

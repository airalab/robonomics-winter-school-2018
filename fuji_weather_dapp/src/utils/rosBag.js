import { open } from 'rosbag'

export default (data, cb, options = {}) => {
  return open(data)
    .then((bag) => {
      bag.readMessages(options, (result) => {
        cb(result)
      })
    })
}

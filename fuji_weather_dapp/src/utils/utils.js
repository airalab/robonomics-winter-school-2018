import axios from 'axios'

export const formatDecimals = (price, decimals) => {
  const priceNum = new web3.BigNumber(price)
  return priceNum.shift(-decimals).toString(10)
}

export const watchTx = (tx) => {
  const transactionReceiptAsync = (resolve, reject) => {
    web3.eth.getTransactionReceipt(tx, (error, receipt) => {
      if (error) {
        reject(error)
      } else if (receipt === null) {
        setTimeout(() => transactionReceiptAsync(resolve, reject), 5000)
      } else {
        resolve(receipt)
      }
    })
  }
  if (Array.isArray(tx)) {
    return Promise.all(tx.map(oneTx => watchTx(oneTx)))
  } else if (typeof tx === 'string') {
    return new Promise(transactionReceiptAsync)
  }
  throw new Error(`Invalid Type: ${tx}`)
}

export const getModel = () => {
  const username = new URL(location.href)
  const url = 'https://raw.githubusercontent.com/' + username.hostname.split('.')[0] + '/' + username.pathname.replace(new RegExp('/', 'g'), '') + '/master/fuji_weather_dapp/model.txt'
  return axios.get(url)
    .then(r => r.data.trim())
    .catch(() => {
      return 'Qmd6bn2JGW26hSx7g5gVCmfgB7uigRPrhAukJn77ee3bMM'
    })
}

export const getAgents = () => {
  const username = new URL(location.href)
  const url = 'https://raw.githubusercontent.com/' + username.hostname.split('.')[0] + '/' + username.pathname.replace(new RegExp('/', 'g'), '') + '/master/fuji_weather_dapp/agents.txt'
  return axios.get(url)
    .then(r => {
      const data = r.data.trim()
      const list = []
      data.split('\n').forEach((v) => {
        const address = web3.toChecksumAddress(v.trim().substr(0, 42))
        if (web3.isAddress(address)) {
          list.push(address)
        }
      })
      return {
        url: 'https://github.com/' + username.hostname.split('.')[0] + '/' + username.pathname.replace(new RegExp('/', 'g'), '') + '/blob/master/fuji_weather_dapp/agents.txt',
        list
      }
    })
    .catch(() => {
      return {
        url: '',
        list: []
      }
    })
}

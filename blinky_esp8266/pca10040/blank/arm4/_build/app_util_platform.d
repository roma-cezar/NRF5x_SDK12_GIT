xchange_mtu_request_req_dec for request decoding.
 *
 * @param[in]      return_code    Return code indicating if command was successful or not.
 * @param[in]      p_buf          Pointer to buffer where encoded data command response will be
 *                                returned.
 * @param[in, out] p_buf_len      \c in: size of \p p_buf buffer.
 *                                \c out: Length of encoded command response packet.
 *
 * @retval NRF_SUCCESS                Encoding success.
 * @retval NRF_ERROR_NULL             Encoding failure. NULL pointer supplied.
 * @retval NRF_ERROR_INVALID_LENGTH   Encoding failure. Incorrect buffer length.
 */
uint32_t ble_gattc_exchange_mtu_request_rsp_enc(uint32_t         return_code,
                                                uint8_t * const  p_buf,
                                                uint32_t * const p_buf_len);

/** @} */

#ifdef __cplusplus
}
#endif

#endif

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                      
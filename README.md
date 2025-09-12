# Robot Chat omniwheel





	          while (read_index != write_index) {
	              // Recherche de l'entête principale
	              if (circular_buffer[read_index] == 0x02 &&
	                  circular_buffer[(read_index + 1) % CIRC_BUF_SIZE] == 0x81)
	              {
	                  data_index = 0;

	                  while (data_index < TRAME_SIZE) {
	                      // Vérifie s’il reste au moins 5 octets
	                      uint16_t available = (write_index >= read_index)
	                          ? (write_index - read_index)
	                          : (CIRC_BUF_SIZE - read_index + write_index);

	                      if (available < 5) {
	                          break; // attendre plus de données
	                      }

	                      // Copie 5 octets
	                      for (int i = 0; i < 5; i++) {
	                          temp5[i] = circular_buffer[(read_index + i) % CIRC_BUF_SIZE];
	                      }

	                      // Confirmation TX ? (trame parasite)
	                      if (temp5[0] == 0x02 && temp5[1] == 0x40 && temp5[2] == 0x01) {
	                          // Skip trame de confirmation
	                          read_index = (read_index + 5) % CIRC_BUF_SIZE;
	                          continue;
	                      }

	                      // Sinon : partie utile, on ajoute à la trame
	                      for (int i = 0; i < 5 && data_index < TRAME_SIZE; i++) {
	                          temp_trame[data_index++] = temp5[i];
	                      }

	                      read_index = (read_index + 5) % CIRC_BUF_SIZE;
	                  }

	                  if (data_index == TRAME_SIZE) {

	                	  if (tarvos_checksum(temp_trame, TRAME_SIZE) == temp_trame[TRAME_SIZE - 1]) {

#ifdef PARTIE_HAUT
	                      memcpy(tarvos_DATA, temp_trame, TRAME_SIZE);
	                      decode_payload(&OTHERData, tarvos_DATA);  // ou un seul struct global


#endif

#ifdef PARTIE_BAS
	                      memcpy(tarvos_DATA, temp_trame, TRAME_SIZE);
	                      decode_payload(&TOPData, tarvos_DATA);  // ou un seul struct global


#endif

	                	  }

	                  }
	              } else {
	                  // Entête invalide : skip 1 octet
	                  read_index = (read_index + 1) % CIRC_BUF_SIZE;
	              }
	          }

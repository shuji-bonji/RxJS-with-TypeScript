---
description: "Gli operatori di filtering di RxJS estraggono solo i dati necessari da uno stream in base a condizioni o tempo. Guida alla selezione di operatori per uso come filter, take, skip, debounceTime, throttleTime, distinct, first, last ed esempi d'uso pratici."
---

# Operatori di Filtering

Gli operatori di filtering di RxJS sono strumenti importanti per selezionare solo i dati necessari da uno stream ed evitare che fluiscano dati non necessari.
Ciò migliora significativamente l'efficienza e le prestazioni dell'applicazione.

Gli operatori di filtering sono un gruppo di operatori RxJS che selezionano i valori all'interno di uno stream e fanno passare solo quelli che soddisfano determinate condizioni.
Controllando il flusso di dati ed elaborando solo i valori necessari, è possibile costruire una pipeline di elaborazione dati efficiente.


## Elenco degli operatori
### ◾ Operatori di filtering di base

| Operatore | Descrizione |
|:---|:---|
| [filter](./filter) | Fa passare solo i valori che soddisfano la condizione |
| [take](./take) | Ottiene solo il numero specificato di primi valori |
| [takeLast](./takeLast) | Ottiene il numero specificato di ultimi valori |
| [takeWhile](./takeWhile) | Ottiene valori finché la condizione è soddisfatta |
| [skip](./skip) | Salta il numero specificato di primi valori |
| [skipLast](./skipLast) | Salta il numero specificato di ultimi valori |
| [skipWhile](./skipWhile) | Salta valori finché la condizione è soddisfatta |
| [skipUntil](./skipUntil) | Salta valori finché un altro Observable emette |
| [first](./first) | Ottiene il primo valore o il primo valore che soddisfa la condizione |
| [last](./last) | Ottiene l'ultimo valore o l'ultimo valore che soddisfa la condizione |
| [elementAt](./elementAt) | Ottiene il valore all'indice specificato |
| [find](./find) | Trova il primo valore che soddisfa la condizione |
| [findIndex](./findIndex) | Ottiene l'indice del primo valore che soddisfa la condizione |
| [ignoreElements](./ignoreElements) | Ignora tutti i valori e passa solo complete/error |


### ◾ Operatori di filtering basati sul tempo

| Operatore | Descrizione |
|:---|:---|
| [debounceTime](./debounceTime) | Emette l'ultimo valore quando non ci sono input per il tempo specificato |
| [throttleTime](./throttleTime) | Fa passare il primo valore e ignora nuovi valori per il tempo specificato |
| [auditTime](./auditTime) | Emette l'ultimo valore dopo il tempo specificato |
| [audit](./audit) | Emette l'ultimo valore controllando la durata con un Observable personalizzato |
| [sampleTime](./sampleTime) | Campiona il valore più recente a intervalli di tempo specificati |


### ◾ Operatori di filtering basati su condizioni

| Operatore | Descrizione |
|:---|:---|
| [distinct](./distinct) | Rimuove tutti i valori duplicati (emette solo valori unici) |
| [distinctUntilChanged](./distinctUntilChanged) | Rimuove valori duplicati consecutivi |
| [distinctUntilKeyChanged](./distinctUntilKeyChanged) | Rileva solo modifiche di proprietà specifiche |


## Use case pratici

- [Use case pratici](./practical-use-cases.md) presenta esempi pratici che combinano più operatori di filtering (ricerca in tempo reale, scroll infinito, ecc.).
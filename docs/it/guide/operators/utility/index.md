---
description: Gli operatori utility sono un gruppo di operatori ausiliari in RxJS che sono responsabili del controllo degli effetti collaterali, dell'elaborazione con ritardo, della gestione delle subscription, ecc.
---

# Operatori Utility

Gli operatori utility in RxJS sono un gruppo di operatori responsabili dell'**elaborazione ausiliaria degli stream (effetti collaterali, controllo dello stato, supporto UI, ecc.)** piuttosto che dello scopo principale di conversione o filtraggio dei dati.

In questa pagina, gli operatori sono categorizzati per scopo come mostrato di seguito, e viene fornito un elenco per confermare il loro utilizzo base.
Per utilizzo dettagliato ed esempi pratici, fare riferimento alle rispettive pagine o ai [Casi d'Uso Pratici](./practical-use-cases.md).


## Elenco Operatori (per Scopo)

### ◾ Effetti Collaterali e Controllo Stato

| Operatore | Descrizione | Spesso Combinato Con |
|--------------|------|------------------|
| [tap](./tap.md) | Esegui effetti collaterali senza modificare i valori (output log, aggiornamenti UI, ecc.) | `map`, `switchMap` |
| [finalize](./finalize.md) | Esegui elaborazione di pulizia quando lo stream termina | `tap`, `catchError` |


### ◾ Controllo Timing e Ritardo

| Operatore | Descrizione | Spesso Combinato Con |
|--------------|------|------------------|
| [delay](./delay.md) | Ritarda l'emissione di ogni valore di un tempo specificato | `tap`, `concatMap` |
| [timeout](./timeout.md) | Genera un errore se l'emissione supera un certo tempo | `catchError`, `retry` |
| [takeUntil](./takeUntil.md) | Termina la subscription quando l'Observable specificato notifica | `interval`, `fromEvent` |


### ◾ Valore Iniziale, Ripetizione, Conversione Array, ecc.

| Operatore | Descrizione | Spesso Combinato Con |
|--------------|------|------------------|
| [startWith](./startWith.md) | Emetti un valore iniziale all'inizio dello stream | `scan`, `combineLatest` |
| [repeat](./repeat.md) | Ri-sottoscrivi l'intero stream dopo il completamento | `tap`, `delay` |
| [retry](./retry.md) | Ritenta in caso di errore | `catchError`, `switchMap` |
| [toArray](./toArray.md) | Emetti tutti i valori nello stream come un singolo array (al completamento) | `concatMap`, `take` |


## Note

- Differenza tra `retry` e `repeat`:
  - `retry`: **Ritenta in caso di errore**
  - `repeat`: **Ritenta in caso di completamento con successo**
- `toArray` non emette un valore a meno che non completi, quindi è comunemente usato con `take()` e simili.

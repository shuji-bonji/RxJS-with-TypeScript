---
description: L'operatore takeUntil è usato per sottoscriversi all'Observable originale fino a quando l'Observable notifier emette un valore e poi disiscriversi quando notificato.
---

# takeUntil

L'operatore `takeUntil` **continua a sottoscrivere l'Observable originale fino a quando l'Observable specificato (trigger di notifica) emette il suo primo valore**. L'Observable originale viene disiscritto nel momento in cui il trigger di notifica emette.

## 🔁 Sintassi Base

```ts
source$.pipe(
  takeUntil(notifier$)
)
```

- `source$`: Observable originale (target della subscription)
- `notifier$`: Observable che segnala lo stop (la subscription si ferma quando questo Observable emette il suo primo valore)

[🌐 Documentazione Ufficiale RxJS - takeUntil](https://rxjs.dev/api/index/function/takeUntil)

## 🧪 Esempio di Utilizzo: Ferma Subscription al Click del Bottone

```ts
import { interval, fromEvent } from 'rxjs';
import { takeUntil } from 'rxjs';

const stopButton = document.createElement('button');
stopButton.textContent = 'stop';
document.body.appendChild(stopButton)

const stop$ = fromEvent(stopButton, 'click');
const source$ = interval(1000); // Emetti numero ogni secondo

source$
  .pipe(takeUntil(stop$))
  .subscribe((val) => console.log(`Valore: ${val}`));
```

📌 Quando `stopButton` viene cliccato, la subscription a `source$` si ferma in quel momento.

## ✅ Casi d'Uso Comuni

- Quando vuoi fermare richieste HTTP o processi di polling con un bottone di annullamento
- Quando vuoi disiscrivere un componente secondo il suo lifecycle
- Quando vuoi terminare l'elaborazione asincrona per transizione pagina o unmounting

## 🔗 Operatori Correlati

- `take`: Prendi valori fino a un certo numero di volte
- `first`: Recupera solo il primo caso ed esci
- `skipUntil`: Ignora fino a quando un particolare Observable emette un valore

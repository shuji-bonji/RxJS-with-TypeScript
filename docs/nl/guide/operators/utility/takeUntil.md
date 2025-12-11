---
description: De takeUntil operator wordt gebruikt om op de originele Observable te abonneren totdat de notifier Observable een waarde uitzendt en vervolgens af te melden wanneer er een melding is.
---

# takeUntil

De `takeUntil` operator **blijft abonneren op de originele Observable totdat de gespecificeerde Observable (meldingstrigger) zijn eerste waarde uitzendt**. De originele Observable wordt afgemeld op het moment dat de meldingstrigger uitzendt.

## 🔁 Basissyntax

```ts
source$.pipe(
  takeUntil(notifier$)
)
```

- `source$`: Originele Observable (abonnementsdoel)
- `notifier$`: Observable die stop signaleert (abonnement stopt wanneer deze Observable zijn eerste waarde uitzendt)

[🌐 RxJS Officiële Documentatie - takeUntil](https://rxjs.dev/api/index/function/takeUntil)

## 🧪 Gebruiksvoorbeeld: Stop abonnement bij knopklik

```ts
import { interval, fromEvent } from 'rxjs';
import { takeUntil } from 'rxjs';

const stopButton = document.createElement('button');
stopButton.textContent = 'stop';
document.body.appendChild(stopButton)

const stop$ = fromEvent(stopButton, 'click');
const source$ = interval(1000); // Geef elke seconde een nummer uit

source$
  .pipe(takeUntil(stop$))
  .subscribe((val) => console.log(`Waarde: ${val}`));
```

📌 Wanneer er op `stopButton` wordt geklikt, stopt het abonnement op `source$` op dat moment.

## ✅ Veelvoorkomende use cases

- Wanneer u HTTP-verzoeken of polling-processen wilt stoppen met een annuleerknop
- Wanneer u een component wilt afmelden volgens zijn levenscyclus
- Wanneer u asynchrone verwerking wilt beëindigen door paginatransitie of unmounting

## 🔗 Gerelateerde operators

- `take`: Neem waarden tot een bepaald aantal keren
- `first`: Haal alleen het eerste geval op en stop
- `skipUntil`: Negeer totdat een bepaalde Observable een waarde uitzendt

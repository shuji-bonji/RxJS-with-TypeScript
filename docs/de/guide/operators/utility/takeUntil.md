---
description: Der takeUntil-Operator wird verwendet, um das ursprüngliche Observable zu abonnieren, bis ein benachrichtigendes Observable einen Wert ausgibt.
---

# takeUntil

`takeUntil` ist ein Operator, der **das ursprüngliche Observable weiter abonniert, bis ein angegebenes Observable (Benachrichtigungs-Trigger) seinen ersten Wert ausgibt**. Sobald der Benachrichtigungs-Trigger einen Wert ausgibt, wird das Abonnement des ursprünglichen Observables beendet.

## 🔁 Grundlegende Syntax

```ts
source$.pipe(
  takeUntil(notifier$)
)
```

- `source$`: Das ursprüngliche Observable (Abonnementziel)
- `notifier$`: Das Observable, das das Stoppen signalisiert (wenn dieses Observable seinen ersten Wert ausgibt, wird das Abonnement beendet)

[🌐 Offizielle RxJS-Dokumentation - takeUntil](https://rxjs.dev/api/index/function/takeUntil)

## 🧪 Verwendungsbeispiel: Abonnement durch Button-Klick stoppen

```ts
import { interval, fromEvent } from 'rxjs';
import { takeUntil } from 'rxjs';

const stopButton = document.createElement('button');
stopButton.textContent = 'stop';
document.body.appendChild(stopButton)

const stop$ = fromEvent(stopButton, 'click');
const source$ = interval(1000); // Gibt jede Sekunde einen Zahlenwert aus

source$
  .pipe(takeUntil(stop$))
  .subscribe((val) => console.log(`Wert: ${val}`));
```

📌 Wenn `stopButton` geklickt wird, wird in diesem Moment das Abonnement von `source$` beendet.

## ✅ Häufige Anwendungsfälle

- Wenn Sie HTTP-Requests oder Polling-Prozesse mit einem Abbrechen-Button stoppen möchten
- Wenn Sie Abonnements entsprechend dem Lebenszyklus einer Komponente beenden möchten
- Wenn Sie asynchrone Verarbeitung bei Seitennavigation oder beim Unmounting abbrechen möchten

## 🔗 Verwandte Operatoren

- `take`: Empfängt Werte bis zu einer bestimmten Anzahl
- `first`: Erhält nur das erste Element und beendet
- `skipUntil`: Ignoriert Werte, bis ein bestimmtes Observable einen Wert ausgibt

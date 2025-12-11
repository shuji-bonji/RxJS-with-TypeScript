---
description: De repeat operator herstart de gehele stream een gespecificeerd aantal keren nadat de bron Observable succesvol is voltooid. Het kan worden gebruikt voor periodieke polling, herhalende animaties, en andere situaties die andere controle vereisen dan retry.
---

# repeat - Herhaal stream

De `repeat` operator herstart de gehele stream een gespecificeerd aantal keren nadat de bron Observable **succesvol is voltooid**.
Dit is nuttig voor polling-processen, herhalende animaties, en controles die anders zijn dan retries.

## 🔰 Basissyntax en werking

Het eenvoudigste gebruik is om een reeks waarden een bepaald aantal keren te laten herhalen.

```ts
import { of } from 'rxjs';
import { repeat } from 'rxjs';

of('A', 'B')
  .pipe(
    repeat(2) // Herhaal gehele reeks 2 keer (geef 2 keer totaal uit)
  )
  .subscribe(console.log);
// Uitvoer:
// A
// B
// A
// B
```

[🌐 RxJS Officiële Documentatie - repeat](https://rxjs.dev/api/index/function/repeat)

## 💡 Typisch gebruiksvoorbeeld

Bijvoorbeeld, het wordt gebruikt voor eenvoudige polling-processen of herhalende weergave-animaties.

```ts
import { of } from 'rxjs';
import { tap, delay, repeat } from 'rxjs';

of('✅ Data succesvol opgehaald')
  .pipe(
    tap(() => console.log('Verzoek gestart')),
    delay(1000),
    repeat(3) // Herhaal 3 keer
  )
  .subscribe(console.log);
// Uitvoer:
// Verzoek gestart
// ✅ Data succesvol opgehaald
// main.ts:6 Verzoek gestart
// ✅ Data succesvol opgehaald
// main.ts:6 Verzoek gestart
// ✅ Data succesvol opgehaald
```

In dit voorbeeld wordt "verzoek → data ophalen" drie keer elke seconde herhaald.

## 🧪 Praktisch codevoorbeeld (met UI)

```ts
import { of } from 'rxjs';
import { repeat, tap } from 'rxjs';

// Uitvoerweergavegebied
const repeatOutput = document.createElement('div');
repeatOutput.innerHTML = '<h3>repeat voorbeeld:</h3>';
document.body.appendChild(repeatOutput);

// Herhalingstelling weergave
let repeatCount = 0;
const repeatCountDisplay = document.createElement('div');
repeatCountDisplay.textContent = `Herhalingstelling: ${repeatCount}`;
repeatCountDisplay.style.fontWeight = 'bold';
repeatOutput.appendChild(repeatCountDisplay);

// Waarden uitvoergebied
const valuesOutput = document.createElement('div');
valuesOutput.style.marginTop = '10px';
valuesOutput.style.padding = '10px';
valuesOutput.style.border = '1px solid #ddd';
valuesOutput.style.maxHeight = '200px';
valuesOutput.style.overflowY = 'auto';
repeatOutput.appendChild(valuesOutput);

// Reeks herhaling
of('A', 'B', 'C')
  .pipe(
    tap(() => {
      repeatCount++;
      repeatCountDisplay.textContent = `Herhalingstelling: ${repeatCount}`;
    }),
    repeat(3)
  )
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Waarde: ${val} (herhaling ${repeatCount})`;
    valuesOutput.appendChild(valueItem);
  });

```

## ✅ Samenvatting

- `repeat` **herstart de gehele Observable na succesvolle voltooiing**
- In tegenstelling tot `retry`, wordt het **niet opnieuw uitgevoerd bij fout**
- Kan worden gebruikt voor herhalende animaties, zoals polling-processen en **knipperende plaatshouders**

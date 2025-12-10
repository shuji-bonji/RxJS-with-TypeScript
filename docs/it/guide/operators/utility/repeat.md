---
description: L'operatore repeat riesegue l'intero stream un numero specificato di volte dopo che l'Observable sorgente completa con successo. Può essere usato per polling periodico, animazione ripetitiva e altre situazioni che richiedono un controllo diverso da retry.
---

# repeat - Ripeti Stream

L'operatore `repeat` riesegue l'intero stream un numero specificato di volte dopo che l'Observable sorgente è **completato con successo**.
È utile per processi di polling, animazioni ripetute e controlli diversi dai retry.

## 🔰 Sintassi e Operazione Base

L'utilizzo più semplice è configurare una sequenza di valori da ripetere un certo numero di volte.

```ts
import { of } from 'rxjs';
import { repeat } from 'rxjs';

of('A', 'B')
  .pipe(
    repeat(2) // Ripeti intera sequenza 2 volte (output 2 volte totali)
  )
  .subscribe(console.log);
// Output:
// A
// B
// A
// B
```

[🌐 Documentazione Ufficiale RxJS - repeat](https://rxjs.dev/api/index/function/repeat)

## 💡 Esempio di Utilizzo Tipico

Per esempio, è usato per semplici processi di polling o animazioni di visualizzazione ripetute.

```ts
import { of } from 'rxjs';
import { tap, delay, repeat } from 'rxjs';

of('✅ Dati recuperati con successo')
  .pipe(
    tap(() => console.log('Richiesta avviata')),
    delay(1000),
    repeat(3) // Ripeti 3 volte
  )
  .subscribe(console.log);
// Output:
// Richiesta avviata
// ✅ Dati recuperati con successo
// Richiesta avviata
// ✅ Dati recuperati con successo
// Richiesta avviata
// ✅ Dati recuperati con successo
```

In questo esempio, "richiesta → recupero dati" viene ripetuto tre volte ogni secondo.

## 🧪 Esempio di Codice Pratico (con UI)

```ts
import { of } from 'rxjs';
import { repeat, tap } from 'rxjs';

// Area di visualizzazione output
const repeatOutput = document.createElement('div');
repeatOutput.innerHTML = '<h3>Esempio repeat:</h3>';
document.body.appendChild(repeatOutput);

// Visualizzazione conteggio ripetizioni
let repeatCount = 0;
const repeatCountDisplay = document.createElement('div');
repeatCountDisplay.textContent = `Conteggio ripetizioni: ${repeatCount}`;
repeatCountDisplay.style.fontWeight = 'bold';
repeatOutput.appendChild(repeatCountDisplay);

// Area output valori
const valuesOutput = document.createElement('div');
valuesOutput.style.marginTop = '10px';
valuesOutput.style.padding = '10px';
valuesOutput.style.border = '1px solid #ddd';
valuesOutput.style.maxHeight = '200px';
valuesOutput.style.overflowY = 'auto';
repeatOutput.appendChild(valuesOutput);

// Ripetizione sequenza
of('A', 'B', 'C')
  .pipe(
    tap(() => {
      repeatCount++;
      repeatCountDisplay.textContent = `Conteggio ripetizioni: ${repeatCount}`;
    }),
    repeat(3)
  )
  .subscribe((val) => {
    const valueItem = document.createElement('div');
    valueItem.textContent = `Valore: ${val} (ripetizione ${repeatCount})`;
    valuesOutput.appendChild(valueItem);
  });

```

## ✅ Riepilogo

- `repeat` **riesegue l'intero Observable dopo completamento con successo**
- A differenza di `retry`, **non riesegue in caso di errore**
- Può essere usato per animazioni ripetitive, come processi di polling e **placeholder lampeggianti**

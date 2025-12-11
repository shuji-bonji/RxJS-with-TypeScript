---
description: De delayWhen operator controleert dynamisch de vertragingstiming van elke waarde met een aparte Observable om flexibele vertragingsverwerking volgens voorwaarden te bereiken.
---

# delayWhen - Dynamische vertragingscontrole

De `delayWhen` operator bepaalt dynamisch de vertragingstijd voor elke waarde **met een individuele Observable**. Terwijl de `delay` operator een vaste tijdvertraging biedt, kan `delayWhen` een verschillende vertraging toepassen voor elke waarde.

## 🔰 Basissyntax en werking

Specificeert een functie die een Observable retourneert die de vertraging voor elke waarde bepaalt.

```ts
import { of, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

of('A', 'B', 'C')
  .pipe(
    delayWhen(value => {
      const delayTime = value === 'B' ? 2000 : 1000;
      return timer(delayTime);
    })
  )
  .subscribe(console.log);
// Uitvoer:
// A (na 1 seconde)
// C (na 1 seconde)
// B (na 2 seconden)
```

In dit voorbeeld krijgt alleen de waarde `'B'` een vertraging van 2 seconden toegepast, de anderen krijgen een vertraging van 1 seconde.

[🌐 RxJS Officiële Documentatie - delayWhen](https://rxjs.dev/api/index/function/delayWhen)

## 💡 Typische gebruiksvoorbeelden

- **Waarde-gebaseerde vertraging**: Verander vertraging op basis van prioriteit of type
- **Vertraging gebaseerd op externe gebeurtenissen**: Wacht op gebruikersinteractie of voltooiing van andere streams
- **Conditionele vertraging**: Vertraging alleen voor een specifieke waarde
- **Asynchrone timing controle**: Wacht op API-respons of data-gereedheid

## 🧪 Praktisch codevoorbeeld 1: Vertraging op prioriteit

Dit is een voorbeeld van het controleren van verwerkingstiming volgens taakprioriteit.

```ts
import { from, timer } from 'rxjs';
import { delayWhen } from 'rxjs';

// UI creatie
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'delayWhen - Prioriteit-gebaseerde vertraging';
container.appendChild(title);

const output = document.createElement('div');
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.maxHeight = '300px';
output.style.overflow = 'auto';
container.appendChild(output);

interface Task {
  id: number;
  name: string;
  priority: 'high' | 'medium' | 'low';
}

const tasks: Task[] = [
  { id: 1, name: 'Taak A', priority: 'low' },
  { id: 2, name: 'Taak B', priority: 'high' },
  { id: 3, name: 'Taak C', priority: 'medium' },
  { id: 4, name: 'Taak D', priority: 'high' },
  { id: 5, name: 'Taak E', priority: 'low' }
];

function addLog(message: string, color: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.style.backgroundColor = color;

  const now = new Date();
  const time = now.toLocaleTimeString('nl-NL', { hour12: false }) +
    '.' + now.getMilliseconds().toString().padStart(3, '0');

  logItem.textContent = `[${time}] ${message}`;
  output.appendChild(logItem);
}

addLog('Verwerking gestart', '#e3f2fd');

from(tasks)
  .pipe(
    delayWhen(task => {
      // Stel vertragingstijd in volgens prioriteit
      let delayTime: number;
      switch (task.priority) {
        case 'high':
          delayTime = 500;  // Hoge prioriteit: 0,5 seconden
          break;
        case 'medium':
          delayTime = 1500; // Gemiddelde prioriteit: 1,5 seconden
          break;
        case 'low':
          delayTime = 3000; // Lage prioriteit: 3 seconden
          break;
      }
      return timer(delayTime);
    })
  )
  .subscribe({
    next: task => {
      const colors = {
        high: '#c8e6c9',
        medium: '#fff9c4',
        low: '#ffccbc'
      };
      addLog(
        `Verwerken ${task.name} (prioriteit: ${task.priority})`,
        colors[task.priority]
      );
    },
    complete: () => {
      addLog('Alle taken voltooid', '#e3f2fd');
    }
  });
```

- Hoge prioriteit taken worden na 0,5 seconden verwerkt
- Gemiddelde prioriteit taken worden na 1,5 seconden verwerkt, lage prioriteit na 3 seconden
- Realiseert verwerkingsvolgorde volgens taakbelang

## 🧪 Praktisch codevoorbeeld 2: Vertraging door externe gebeurtenissen

Dit is een voorbeeld van het wachten op een gebruikersklik voordat een waarde wordt uitgegeven.

```ts
import { of, fromEvent } from 'rxjs';
import { delayWhen, take, tap } from 'rxjs';

// UI creatie
const container2 = document.createElement('div');
container2.style.marginTop = '20px';
document.body.appendChild(container2);

const title2 = document.createElement('h3');
title2.textContent = 'delayWhen - Wachten op klik';
container2.appendChild(title2);

const button = document.createElement('button');
button.textContent = 'Klik om volgende waarde te tonen';
button.style.marginBottom = '10px';
container2.appendChild(button);

const output2 = document.createElement('div');
output2.style.border = '1px solid #ccc';
output2.style.padding = '10px';
output2.style.minHeight = '100px';
container2.appendChild(output2);

function addLog2(message: string) {
  const logItem = document.createElement('div');
  logItem.style.padding = '5px';
  logItem.style.marginBottom = '3px';
  logItem.textContent = message;
  output2.appendChild(logItem);
}

let clickCount = 0;

of('Bericht 1', 'Bericht 2', 'Bericht 3')
  .pipe(
    tap(msg => {
      addLog2(`Wachten: ${msg} (klik op de knop)`);
      button.textContent = `Klik om "${msg}" te tonen`;
    }),
    delayWhen(() => {
      // Vertraag tot klikgebeurtenis optreedt
      return fromEvent(button, 'click').pipe(take(1));
    })
  )
  .subscribe({
    next: msg => {
      clickCount++;
      addLog2(`✅ Getoond: ${msg}`);
      if (clickCount < 3) {
        button.disabled = false;
      } else {
        button.textContent = 'Voltooid';
        button.disabled = true;
      }
    },
    complete: () => {
      addLog2('--- Alle berichten getoond ---');
    }
  });
```

- Elke waarde wordt uitgegeven na wachten op een gebruikersklik
- Vertragingscontrole getriggerd door externe gebeurtenissen is mogelijk
- Kan worden toegepast op interactieve sequentieverwerking

## 🆚 Vergelijking met delay

```ts
import { of, timer } from 'rxjs';
import { delay, delayWhen } from 'rxjs';

// delay - vaste tijdvertraging
of(1, 2, 3)
  .pipe(delay(1000))
  .subscribe(console.log);
// Alle waarden vertraagd met 1 seconde

// delayWhen - verschillende vertraging per waarde
of(1, 2, 3)
  .pipe(
    delayWhen(value => timer(value * 1000))
  )
  .subscribe(console.log);
// 1 na 1 seconde, 2 na 2 seconden, 3 na 3 seconden
```

| Operator | Vertragingscontrole | Gebruiksscenario |
|:---|:---|:---|
| `delay` | Vaste tijd | Eenvoudige uniforme vertraging |
| `delayWhen` | Dynamisch (per waarde) | Conditionele vertraging, wachten op externe gebeurtenis |

## ⚠️ Belangrijke opmerkingen

### 1. Vertraging Observable wordt elke keer nieuw gegenereerd

```ts
// ❌ Slecht voorbeeld: Dezelfde Observable-instantie hergebruiken
const delayObs$ = timer(1000);
source$.pipe(
  delayWhen(() => delayObs$)  // Werkt niet vanaf 2e keer
).subscribe();

// ✅ Goed voorbeeld: Genereer elke keer nieuwe Observable
source$.pipe(
  delayWhen(() => timer(1000))
).subscribe();
```

### 2. Wanneer vertraging Observable niet voltooit

```ts
import { of, NEVER } from 'rxjs';
import { delayWhen } from 'rxjs';

// ❌ Slecht voorbeeld: NEVER retourneren vertraagt voor altijd
of(1, 2, 3)
  .pipe(
    delayWhen(() => NEVER)  // Waarden worden niet uitgegeven
  )
  .subscribe(console.log);
// Niets uitgevoerd
```

Vertraging Observable moet altijd een waarde uitzenden of voltooien.

### 3. Foutafhandeling

Als er een fout optreedt binnen de vertraging Observable, zal de hele stream een fout geven.

```ts
import { of, throwError, timer, delayWhen } from 'rxjs';

of(1, 2, 3)
  .pipe(
    delayWhen(value => {
      if (value === 2) {
        return throwError(() => new Error('Vertragingsfout'));
      }
      return timer(1000);
    })
  )
  .subscribe({
    next: console.log,
    error: err => console.error('Fout:', err.message)
  });
// Uitvoer: 1
// Fout: Vertragingsfout
```

## 📚 Gerelateerde operators

- **[delay](./delay)** - Vaste tijdvertraging
- **[debounceTime](../filtering/debounceTime)** - Vertraging nadat invoer stopt
- **[throttleTime](../filtering/throttleTime)** - Laat waarde door elke vaste periode
- **[timeout](./timeout)** - Timeout-controle

## ✅ Samenvatting

De `delayWhen` operator controleert dynamisch de vertragingstiming voor elke waarde.

- ✅ Verschillende vertragingen kunnen worden toegepast op elke waarde
- ✅ Vertragingscontrole door externe gebeurtenissen en Observable
- ✅ Pas verwerkingstiming aan op basis van prioriteit en type
- ⚠️ Vertraging Observable moet elke keer nieuw worden gegenereerd
- ⚠️ Vertraging Observable moet voltooien of een waarde uitzenden
- ⚠️ Wees voorzichtig met foutafhandeling

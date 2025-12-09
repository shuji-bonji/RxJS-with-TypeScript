---
description: "Die forkJoin-Erstellungsfunktion gibt den jeweils letzten Wert von mehreren Observables zusammen als Array oder Objekt aus, nachdem sie alle abgeschlossen wurden. Ideal wenn Sie die Ergebnisse paralleler API-Aufrufe auf einmal abrufen möchten."
---

# forkJoin - Alle letzten Werte gemeinsam ausgeben

`forkJoin` ist eine Erstellungsfunktion, die **nachdem mehrere Observables alle abgeschlossen sind, den letzten Wert jeder Observable zusammen als Array oder Objekt** ausgibt.
Dies ist sehr nützlich, wenn man alle Ergebnisse zusammen verwenden möchte, nachdem sie alle fertiggestellt wurden.

## Grundlegende Syntax und Verwendung

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

const user$ = of('Benutzer A').pipe(delay(1000));
const posts$ = of('Beitragsliste').pipe(delay(1500));

forkJoin([user$, posts$]).subscribe(([user, posts]) => {
  console.log(user, posts);
});

// Ausgabe:
// Benutzer A Beitragsliste
```

- Wartet, bis alle Observables `complete` sind.
- Nur der **letzte ausgegebene Wert** jeder Observable wird zusammengefasst und ausgegeben.

[🌐 Offizielle RxJS-Dokumentation - forkJoin](https://rxjs.dev/api/index/function/forkJoin)

## Typische Nutzungsmuster

- **Mehrere API-Anfragen parallel ausführen und alle Ergebnisse zusammenfassen**
- **Mehrere zum ersten Ladezeitpunkt benötigte Datensätze in einem Durchgang abrufen**
- **Alle relevanten Daten auf einmal abrufen und den Bildschirm auf einmal zeichnen**

## Praktisches Code-Beispiel (mit UI)

Simuliert mehrere API-Anfragen und zeigt sie gemeinsam an, wenn alle Ergebnisse verfügbar sind.

```ts
import { forkJoin, of } from 'rxjs';
import { delay } from 'rxjs';

// Ausgabebereich erstellen
const output = document.createElement('div');
output.innerHTML = '<h3>forkJoin Praxisbeispiel:</h3>';
document.body.appendChild(output);

// Dummy-Datenströme
const user$ = of({ id: 1, name: 'Max Mustermann' }).pipe(delay(2000));
const posts$ = of([{ id: 1, title: 'Beitrag 1' }, { id: 2, title: 'Beitrag 2' }]).pipe(delay(1500));
const weather$ = of({ temp: 22, condition: 'Sonnig' }).pipe(delay(1000));

// Lademeldung
const loading = document.createElement('div');
loading.textContent = 'Daten werden geladen...';
loading.style.color = 'blue';
output.appendChild(loading);

// Alle Anfragen abgeschlossen, dann gemeinsam ausgeben
forkJoin({
  user: user$,
  posts: posts$,
  weather: weather$
}).subscribe(result => {
  output.removeChild(loading);

  const pre = document.createElement('pre');
  pre.textContent = JSON.stringify(result, null, 2);
  pre.style.background = '#f5f5f5';
  pre.style.padding = '10px';
  pre.style.borderRadius = '5px';
  output.appendChild(pre);

  const summary = document.createElement('div');
  summary.textContent = `Benutzer: ${result.user.name}, Wetter: ${result.weather.condition}, Beiträge: ${result.posts.length}`;
  output.appendChild(summary);
});
```

- Zeigt zuerst die Lademeldung an,
- Wenn alle Daten bereit sind, werden die Ergebnisse zusammen angezeigt.

## Verwandte Operatoren

- **[combineLatest](/de/guide/creation-functions/combination/combineLatest)** - Kombiniert neueste Werte (gibt während des Streams aus)
- **[zip](/de/guide/creation-functions/combination/zip)** - Paart entsprechende Werte (Erstellungsfunktion)
- **[merge](/de/guide/creation-functions/combination/merge)** - Führt Streams parallel zusammen (Erstellungsfunktion)

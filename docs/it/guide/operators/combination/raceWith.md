---
description: "raceWith è un operatore di join di RxJS che adotta solo il flusso che ha emesso per primo il valore tra l'Observable originale e gli altri Observable. È ideale per implementazioni di timeout, processi di fallback, adozione più rapida da più fonti di dati e altre situazioni in cui è richiesta una selezione per condizioni di gara; la versione dell'operatore pipe è comoda per l'uso in pipeline."
---

# raceWith - adotta il flusso più veloce

L'operatore raceWith **adotta** solo il primo Observable che emette un valore dall'Observable originale e dagli altri Observable specificati,
in seguito ignora gli altri Observable.
Si tratta della versione Pipeable Operator di Creation Function `race`.

## 🔰 Sintassi e utilizzo di base

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Lentamente. (5Secondi)'));
const medium$ = timer(3000).pipe(map(() => 'Normale (3Secondi)'));
const fast$ = timer(2000).pipe(map(() => 'Veloce (2Secondi)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Uscita: Veloce (2Secondi)
```

- Solo l'Observable che ha emesso il valore per primo (`fast$` in questo esempio) è il vincitore e continua con i flussi successivi.
- Gli altri Observable vengono ignorati.

[🌐 Documentazione ufficiale di RxJS - raceWith](https://rxjs.dev/api/operators/raceWith)

## 💡 Tipico modello di utilizzo.

- **Implementazione del timeout**: gareggia con il timer del timeout contro il processo principale.
- **Elaborazione di ritorno**: adottare il più veloce tra più fonti di dati.
- **Interazione con l'utente**: adottare il più veloce dei clic e dell'avanzamento automatico.

## 🧠 Esempi pratici di codice (con UI)

Questo è un esempio di gara tra il clic manuale e il timer di avanzamento automatico e l'adozione di quello più veloce.

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

// Creazione dell'area di uscita
const output = document.createElement('div');
output.innerHTML = '<h3>raceWith Esempi pratici di:</h3>';
document.body.appendChild(output);

// Creazione dei pulsanti
const button = document.createElement('button');
button.textContent = 'Procedere manualmente (5Fare clic entro pochi secondi)';
document.body.appendChild(button);

// Messaggio di attesa
const waiting = document.createElement('div');
waiting.textContent = '5Fare clic sul pulsante entro pochi secondi o attendere l'avanzamento automatico...';
waiting.style.marginTop = '10px';
output.appendChild(waiting);

// Flusso di clic manuale
const manualClick$ = fromEvent(button, 'click').pipe(
  take(1),
  map(() => '👆 È stato selezionato il clic manuale！')
);

// Timer di avanzamento automatico (5(dopo 2,5 secondi)
const autoProgress$ = timer(5000).pipe(
  map(() => '⏰ È stato selezionato l'avanzamento automatico！')
);

// Esecuzione della gara
manualClick$
  .pipe(raceWith(autoProgress$))
  .subscribe((winner) => {
    waiting.remove();
    button.disabled = true;

    const result = document.createElement('div');
    result.innerHTML = `<strong>${winner}</strong>`;
    result.style.color = 'green';
    result.style.fontSize = '18px';
    result.style.marginTop = '10px';
    output.appendChild(result);
  });
```

- Il clic manuale viene adottato se il pulsante viene cliccato entro 5 secondi.
- Dopo 5 secondi, viene adottata la progressione automatica.
- Il più veloce è il vincitore**, il più lento viene ignorato.

## 🔄 Differenze con la Creation Function "race".

### Differenze di base.

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Lentamente. (5Secondi)'));
const medium$ = timer(3000).pipe(map(() => 'Normale (3Secondi)'));
const fast$ = timer(2000).pipe(map(() => 'Veloce (2Secondi)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Uscita: Veloce (2Secondi)
```

### Esempi specifici di utilizzo

**Se si desidera una gara semplice, la Creation Function è la soluzione migliore**.


```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const server1$ = timer(3000).pipe(map(() => 'Server1Risposta da'));
const server2$ = timer(2000).pipe(map(() => 'Server2Risposta da'));
const server3$ = timer(4000).pipe(map(() => 'Server3Risposta da'));

// Semplice e di facile lettura
race(server1$, server2$, server3$).subscribe(response => {
  console.log('Adottato:', response);
});
// Uscita: Adottato: Server2Risposta da (Più veloce2Secondi)
```

**Se si desidera aggiungere un processo di conversione al flusso principale, si consiglia di utilizzare Pipeable Operator**.

```ts
import { fromEvent, timer, of } from 'rxjs';
import { raceWith, map, switchMap, catchError } from 'rxjs';

const searchButton = document.createElement('button');
searchButton.textContent = 'Ricerca';
document.body.appendChild(searchButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Principale: Richieste di ricerca dell'utente
const userSearch$ = fromEvent(searchButton, 'click').pipe(
  switchMap(() => {
    output.textContent = 'Ricerca...';

    // APISimulare una chiamata (3(richiede secondi)
    return timer(3000).pipe(
      map(() => '🔍 Risultati della ricerca: 100Risultati'),
      catchError((err: unknown) => of('❌ Errore verificatosi'))
    );
  })
);

// ✅ Pipeable OperatorEdizione - Completato in una pipeline
userSearch$
  .pipe(
    raceWith(
      // Timeout (in2secondi)
      timer(2000).pipe(
        map(() => '⏱️ Timeout (s): La ricerca richiede troppo tempo')
      )
    )
  )
  .subscribe(result => {
    output.textContent = result;
  });

// ❌ Creation FunctionEdizione - Il mainstream deve essere scritto separatamente
import { race } from 'rxjs';
race(
  userSearch$,
  timer(2000).pipe(
    map(() => '⏱️ Timeout (s): La ricerca richiede troppo tempo')
  )
).subscribe(result => {
  output.textContent = result;
});
```

**Implementazione dell'elaborazione di fallback**

```ts
import { timer, throwError } from 'rxjs';
import { raceWith, map, mergeMap, catchError, delay } from 'rxjs';
import { of } from 'rxjs';

// UICreazione
const output = document.createElement('div');
output.innerHTML = '<h3>Acquisizione dei dati (con fall-back)</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Inizio dell'acquisizione dei dati';
document.body.appendChild(button);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
output.appendChild(statusArea);

button.addEventListener('click', () => {
  statusArea.textContent = 'Durante l'acquisizione...';

  // PrincipaleAPI(priorità)：Tempo々Fallimento
  const mainApi$ = timer(1500).pipe(
    mergeMap(() => {
      const success = Math.random() > 0.5;
      if (success) {
        return of('✅ PrincipaleAPIAcquisizione riuscita da');
      } else {
        return throwError(() => new Error('PrincipaleAPIFallimento'));
      }
    }),
    catchError((err: unknown) => {
      console.log('PrincipaleAPIFallimento, ceduto al fallback...');
      // Ritardo su errore, cedere al fallback
      return of('').pipe(delay(10000));
    })
  );

  // ✅ Pipeable OperatorEdizione - PrincipaleAPIAggiungere il fallback a
  mainApi$
    .pipe(
      raceWith(
        // BackupAPI(Fallback)：Leggermente più lento ma affidabile
        timer(2000).pipe(
          map(() => '🔄 BackupAPIRecuperato da')
        )
      )
    )
    .subscribe(result => {
      if (result) {
        statusArea.textContent = result;
        statusArea.style.color = result.includes('Principale') ? 'green' : 'orange';
      }
    });
});
```

**Adottare il più veloce da più fonti di dati**.

```ts
import { timer, fromEvent } from 'rxjs';
import { raceWith, map, mergeMap } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>MultiploCDNCarico più veloce da</h3>';
document.body.appendChild(output);

const loadButton = document.createElement('button');
loadButton.textContent = 'Libreria di caricamento';
document.body.appendChild(loadButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
output.appendChild(result);

fromEvent(loadButton, 'click').pipe(
  mergeMap(() => {
    result.textContent = 'Caricamento...';

    // CDN1Caricamento (simulato) da
    const cdn1$ = timer(Math.random() * 3000).pipe(
      map(() => ({ source: 'CDN1 (US)', data: 'library.js' }))
    );

    // ✅ Pipeable OperatorEdizione - CDN1nella principale e altriCDNcome concorrenti.
    return cdn1$.pipe(
      raceWith(
        // CDN2Caricamento (simulato) da
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN2 (EU)', data: 'library.js' }))
        ),
        // CDN3Caricamento (simulato) da
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN3 (Asia)', data: 'library.js' }))
        )
      )
    );
  })
).subscribe(response => {
  result.innerHTML = `
    <strong>✅ Caricamento completato</strong><br>
    Acquisito da: ${response.source}<br>
    File: ${response.data}
  `;
  result.style.color = 'green';
});
```

### Riepilogo.

- race`**: ideale se si vuole semplicemente adottare il più veloce tra più flussi.
- RaceWith**: ideale se si vogliono implementare timeout e fallback durante la trasformazione e l'elaborazione del flusso principale.

## ⚠️ Note.

### Esempio di implementazione del timeout

Implementazione della gestione del timeout usando raceWith.

```ts
import { of, timer, throwError } from 'rxjs';
import { raceWith, delay, mergeMap } from 'rxjs';

// Processo che richiede tempo (3secondi)
const slowRequest$ = of('Acquisizione dei dati riuscita').pipe(delay(3000));

// Timeout (in2secondi)
const timeout$ = timer(2000).pipe(
  mergeMap(() => throwError(() => new Error('Timeout (s)')))
);

slowRequest$
  .pipe(raceWith(timeout$))
  .subscribe({
    next: console.log,
    error: err => console.error(err.message)
  });
// Uscita: Timeout (s)
```

### Tutti i flussi sono sottoscritti a

raceWith sottoscrive tutti gli Observable finché non viene deciso un vincitore.
Dopo la decisione del vincitore, l'Observable perdente viene automaticamente cancellato.

```ts
import { timer } from 'rxjs';
import { raceWith, tap, map } from 'rxjs';

const slow$ = timer(3000).pipe(
  tap(() => console.log('slow$ L'accensione')),
  map(() => 'slow')
);

const fast$ = timer(1000).pipe(
  tap(() => console.log('fast$ L'accensione')),
  map(() => 'fast')
);

slow$.pipe(raceWith(fast$)).subscribe(console.log);
// Uscita:
// fast$ L'accensione
// fast
// (slow$è1Non è stata sottoscritta al secondo,3non viene sparato alla fine del secondo periodo.)
```

### Per Observable sincroni.

Se tutto viene emesso in modo sincrono, il primo registrato è il vincitore.

```ts
import { of } from 'rxjs';
import { raceWith } from 'rxjs';

of('A').pipe(
  raceWith(of('B'), of('C'))
).subscribe(console.log);
// Uscita: A (perché è stato sottoscritto per primo)
```

## 📚 Operatori correlati.

- **[race](/it/guide/creation-functions/selezione/race)** - Versione di Creation Function
- **[timeout](/it/guide/operators/utilità/timeout)** - Operatore di solo timeout.
- **[mergeWith](/it/guide/operators/combinazione/mergeWith)** - Unisce tutti i flussi.

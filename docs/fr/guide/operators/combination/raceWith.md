---
description: "raceWith est un opérateur de jointure RxJS qui adopte uniquement le flux qui a émis la valeur en premier parmi l'Observable d'origine et les autres Observables. Il est idéal pour les implémentations de délais, les processus de repli, l'adoption la plus rapide à partir de plusieurs sources de données, et d'autres situations où la sélection par des conditions de course est nécessaire ; la version de l'opérateur pipeable est pratique pour une utilisation dans les pipelines."
---

# raceWith - adopte le flux le plus rapide

L'opérateur `raceWith` **adopte** seulement le premier Observable qui émet une valeur à partir de l'Observable original et des autres Observables spécifiés,
par la suite, il ignore les autres Observable.
C'est la version Pipeable Operator de la `race` de Creation Function.

## 🔰 Syntaxe de base et utilisation

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Lentement. (5secondes)'));
const medium$ = timer(3000).pipe(map(() => 'Normale (3secondes)'));
const fast$ = timer(2000).pipe(map(() => 'Rapide (2secondes)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Sortie: Rapide (2secondes)
```

- Seul l'Observable qui a émis la valeur en premier (`fast$` dans cet exemple) est gagnant et continue avec les flux suivants.
- Les autres Observable sont ignorés.

[🌐 Documentation officielle de RxJS - `raceWith`](https://rxjs.dev/api/operators/raceWith)

## 💡 Modèle d'utilisation typique.

- **Mise en œuvre du délai d'attente** : faire courir le délai d'attente contre le processus principal.
- **Traitement de secours** : adopter le traitement le plus rapide parmi plusieurs sources de données.
- Interaction avec l'utilisateur** : adopter le plus rapide des clics et de l'auto-progression.

## 🧠 Exemples de code pratique (avec interface utilisateur)

Voici un exemple de course entre le clic manuel et la minuterie de progression automatique et l'adoption de la plus rapide.

```ts
import { fromEvent, timer } from 'rxjs';
import { raceWith, map, take } from 'rxjs';

// Création d'une zone de sortie
const output = document.createElement('div');
output.innerHTML = '<h3>raceWith Exemples pratiques de:</h3>';
document.body.appendChild(output);

// Création de boutons
const button = document.createElement('button');
button.textContent = 'Procéder manuellement (5Cliquer dans les secondes qui suivent)';
document.body.appendChild(button);

// Message d'attente
const waiting = document.createElement('div');
waiting.textContent = '5Cliquez sur le bouton dans les secondes qui suivent ou attendez l'avance automatique....';
waiting.style.marginTop = '10px';
output.appendChild(waiting);

// Flux de clics manuels
const manualClick$ = fromEvent(button, 'click').pipe(
  take(1),
  map(() => '👆 Le clic manuel a été sélectionné！')
);

// Minuterie de progression automatique (5(après 2,5 secondes)
const autoProgress$ = timer(5000).pipe(
  map(() => '⏰ La progression automatique a été sélectionnée！')
);

// Exécution de la course
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

- Le clic manuel est adopté si le bouton est cliqué dans les 5 secondes.
- Après 5 secondes, la progression automatique est adoptée.
- Le plus rapide est le gagnant**, le plus lent est ignoré.

## 🔄 Différences avec la Creation Function `race`.

### Différences de base.

```ts
import { timer } from 'rxjs';
import { raceWith, map } from 'rxjs';

const slow$ = timer(5000).pipe(map(() => 'Lentement. (5secondes)'));
const medium$ = timer(3000).pipe(map(() => 'Normale (3secondes)'));
const fast$ = timer(2000).pipe(map(() => 'Rapide (2secondes)'));

slow$
  .pipe(raceWith(medium$, fast$))
  .subscribe(console.log);

// Sortie: Rapide (2secondes)
```

### Exemples spécifiques d'utilisation

**Si vous souhaitez une course simple, Creation Function est la meilleure solution**.


```ts
import { race, timer } from 'rxjs';
import { map } from 'rxjs';

const server1$ = timer(3000).pipe(map(() => 'Serveur1Réponse du'));
const server2$ = timer(2000).pipe(map(() => 'Serveur2Réponse du'));
const server3$ = timer(4000).pipe(map(() => 'Serveur3Réponse du'));

// Simple et facile à lire
race(server1$, server2$, server3$).subscribe(response => {
  console.log('Adopté:', response);
});
// Sortie: Adopté: Serveur2Réponse du (Le plus rapide2secondes)
```

**Si vous voulez ajouter un processus de conversion au flux principal, le Pipeable Operator est recommandé**.

```ts
import { fromEvent, timer, of } from 'rxjs';
import { raceWith, map, switchMap, catchError } from 'rxjs';

const searchButton = document.createElement('button');
searchButton.textContent = 'Recherche';
document.body.appendChild(searchButton);

const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Généraliste: Demandes de recherche des utilisateurs
const userSearch$ = fromEvent(searchButton, 'click').pipe(
  switchMap(() => {
    output.textContent = 'Recherche...';

    // APISimuler un appel (3(prend quelques secondes)
    return timer(3000).pipe(
      map(() => '🔍 Résultats de la recherche: 100Résultats'),
      catchError((err: unknown) => of('❌ Erreur survenue'))
    );
  })
);

// ✅ Pipeable OperatorEdition - Terminé en une seule fois
userSearch$
  .pipe(
    raceWith(
      // Délai d'attente (en2secondes)
      timer(2000).pipe(
        map(() => '⏱️ Délai d'attente (s): La recherche prend trop de temps')
      )
    )
  )
  .subscribe(result => {
    output.textContent = result;
  });

// ❌ Creation FunctionEdition - Le mainstream doit être écrit séparément
import { race } from 'rxjs';
race(
  userSearch$,
  timer(2000).pipe(
    map(() => '⏱️ Délai d'attente (s): La recherche prend trop de temps')
  )
).subscribe(result => {
  output.textContent = result;
});
```

**Mise en œuvre d'un traitement de secours**

```ts
import { timer, throwError } from 'rxjs';
import { raceWith, map, mergeMap, catchError, delay } from 'rxjs';
import { of } from 'rxjs';

// UICréation
const output = document.createElement('div');
output.innerHTML = '<h3>Acquisition de données (avec repli)</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Début de l'acquisition des données';
document.body.appendChild(button);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
output.appendChild(statusArea);

button.addEventListener('click', () => {
  statusArea.textContent = 'Pendant l'acquisition...';

  // PrincipalAPI(Priorité)：Temps々Échec
  const mainApi$ = timer(1500).pipe(
    mergeMap(() => {
      const success = Math.random() > 0.5;
      if (success) {
        return of('✅ PrincipalAPIAcquisition réussie à partir de');
      } else {
        return throwError(() => new Error('PrincipalAPIÉchec'));
      }
    }),
    catchError((err: unknown) => {
      console.log('PrincipalAPIÉchec, cédé au repli...');
      // Retard en cas d'erreur, cédé à la solution de repli
      return of('').pipe(delay(10000));
    })
  );

  // ✅ Pipeable OperatorEdition - PrincipalAPIAjouter le repli à
  mainApi$
    .pipe(
      raceWith(
        // SauvegardeAPI(repli)：Légèrement plus lent mais fiable
        timer(2000).pipe(
          map(() => '🔄 SauvegardeAPIRécupéré à partir de')
        )
      )
    )
    .subscribe(result => {
      if (result) {
        statusArea.textContent = result;
        statusArea.style.color = result.includes('Principal') ? 'green' : 'orange';
      }
    });
});
```

**Adopter le plus rapidement à partir de plusieurs sources de données**.

```ts
import { timer, fromEvent } from 'rxjs';
import { raceWith, map, mergeMap } from 'rxjs';

const output = document.createElement('div');
output.innerHTML = '<h3>MultipleCDNChargement le plus rapide à partir de</h3>';
document.body.appendChild(output);

const loadButton = document.createElement('button');
loadButton.textContent = 'Bibliothèque de chargement';
document.body.appendChild(loadButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
output.appendChild(result);

fromEvent(loadButton, 'click').pipe(
  mergeMap(() => {
    result.textContent = 'Chargement...';

    // CDN1Chargement (simulé) de
    const cdn1$ = timer(Math.random() * 3000).pipe(
      map(() => ({ source: 'CDN1 (US)', data: 'library.js' }))
    );

    // ✅ Pipeable OperatorEdition - CDN1dans le principal et d'autresCDNcomme concurrents.
    return cdn1$.pipe(
      raceWith(
        // CDN2Chargement (simulé) de
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN2 (EU)', data: 'library.js' }))
        ),
        // CDN3Chargement (simulé) de
        timer(Math.random() * 3000).pipe(
          map(() => ({ source: 'CDN3 (Asia)', data: 'library.js' }))
        )
      )
    );
  })
).subscribe(response => {
  result.innerHTML = `
    <strong>✅ Chargement effectué</strong><br>
    Acquis à partir de: ${response.source}<br>
    Dossier: ${response.data}
  `;
  result.style.color = 'green';
});
```

### Résumé.

- **`race`** : idéal si vous voulez simplement adopter le plus rapide parmi plusieurs flux.
- raceWith : idéal si vous voulez implémenter des timeouts et des fallbacks pendant que vous transformez et traitez le flux principal.

## ⚠️ Notes.

### Exemple de mise en œuvre d'un délai d'attente

Implémentation de la gestion du timeout en utilisant `raceWith`.

```ts
import { of, timer, throwError } from 'rxjs';
import { raceWith, delay, mergeMap } from 'rxjs';

// Processus chronophage (3secondes)
const slowRequest$ = of('Acquisition de données réussie').pipe(delay(3000));

// Délai d'attente (en2secondes)
const timeout$ = timer(2000).pipe(
  mergeMap(() => throwError(() => new Error('Délai d'attente (s)')))
);

slowRequest$
  .pipe(raceWith(timeout$))
  .subscribe({
    next: console.log,
    error: err => console.error(err.message)
  });
// Sortie: Délai d'attente (s)
```

### Tous les flux sont abonnés à

`raceWith` abonne tous les Observable jusqu'à ce qu'un gagnant soit désigné.
Une fois le gagnant désigné, l'Observable perdant est automatiquement désabonné.

```ts
import { timer } from 'rxjs';
import { raceWith, tap, map } from 'rxjs';

const slow$ = timer(3000).pipe(
  tap(() => console.log('slow$ Le tir')),
  map(() => 'slow')
);

const fast$ = timer(1000).pipe(
  tap(() => console.log('fast$ Le tir')),
  map(() => 'fast')
);

slow$.pipe(raceWith(fast$)).subscribe(console.log);
// Sortie:
// fast$ Le tir
// fast
// (slow$est1Désabonné à la seconde,3n'est pas tiré à la fin de la deuxième période.)
```

### Pour les Observables synchrones.

Si tout est émis de manière synchrone, le premier enregistré est le gagnant.

```ts
import { of } from 'rxjs';
import { raceWith } from 'rxjs';

of('A').pipe(
  raceWith(of('B'), of('C'))
).subscribe(console.log);
// Sortie: A (parce qu'il a d'abord été abonné à)
```

## 📚 Opérateurs apparentés.

- **[race](/fr/guide/creation-functions/selection/race)** - Version de la Creation Function
- **[timeout](/fr/guide/operators/utility/timeout)** - Opérateur de timeout uniquement.
- **[mergeWith](/fr/guide/operators/combination/mergeWith)** - Fusion de tous les flux

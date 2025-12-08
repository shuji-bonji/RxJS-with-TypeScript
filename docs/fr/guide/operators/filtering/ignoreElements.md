---
description: "L'opérateur ignoreElements est un opérateur de filtrage RxJS qui ignore toutes les valeurs et ne transmet que les notifications de complétion et d'erreur. Utile pour attendre la fin d'un traitement."
---

# ignoreElements - Ignorer toutes les valeurs et ne transmettre que complétion/erreur

L'opérateur `ignoreElements` **ignore toutes les valeurs** émises par l'Observable source et ne transmet que les **notifications de complétion et d'erreur** en aval.

## 🔰 Syntaxe de base et utilisation

```ts
import { of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5);

source$.pipe(
  ignoreElements()
).subscribe({
  next: value => console.log('Valeur:', value), // Non appelé
  complete: () => console.log('Terminé')
});
// Sortie: Terminé
```

**Flux d'opération** :
1. 1, 2, 3, 4, 5 sont tous ignorés
2. Seule la notification de complétion est transmise en aval

[🌐 Documentation officielle RxJS - `ignoreElements`](https://rxjs.dev/api/operators/ignoreElements)

## 💡 Patterns d'utilisation typiques

- **Attente de fin de traitement** : Quand vous n'avez pas besoin des valeurs mais seulement de savoir quand c'est terminé
- **Exécution d'effets de bord uniquement** : Exécuter des effets de bord avec tap et ignorer les valeurs
- **Gestion des erreurs** : Quand vous voulez uniquement capturer les erreurs
- **Synchronisation de séquences** : Attendre la complétion de plusieurs traitements

## 🧠 Exemple de code pratique 1 : Attente de fin d'initialisation

Un exemple d'attente de la complétion de plusieurs processus d'initialisation.

```ts
import { from, forkJoin, of } from 'rxjs';
import { ignoreElements, tap, delay, concat } from 'rxjs';

// Création de l'UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Initialisation de l\'application';
container.appendChild(title);

const statusArea = document.createElement('div');
statusArea.style.marginTop = '10px';
container.appendChild(statusArea);

const completeMessage = document.createElement('div');
completeMessage.style.marginTop = '10px';
completeMessage.style.padding = '10px';
completeMessage.style.display = 'none';
container.appendChild(completeMessage);

// Fonction pour ajouter des logs de statut
function addLog(message: string, color: string = 'black') {
  const log = document.createElement('div');
  log.textContent = `[${new Date().toLocaleTimeString()}] ${message}`;
  log.style.color = color;
  statusArea.appendChild(log);
}

// Initialisation 1: Connexion à la base de données
const initDatabase$ = from(['Connexion DB en cours...', 'Vérification des tables...', 'DB prête']).pipe(
  tap(msg => addLog(msg, 'blue')),
  delay(500),
  ignoreElements() // Ignore les valeurs, ne notifie que la complétion
);

// Initialisation 2: Chargement du fichier de configuration
const loadConfig$ = from(['Chargement de la config...', 'Analyse de la config...', 'Config appliquée']).pipe(
  tap(msg => addLog(msg, 'green')),
  delay(700),
  ignoreElements()
);

// Initialisation 3: Authentification utilisateur
const authenticate$ = from(['Vérification des identifiants...', 'Validation du token...', 'Authentification terminée']).pipe(
  tap(msg => addLog(msg, 'purple')),
  delay(600),
  ignoreElements()
);

// Exécuter tous les processus d'initialisation
addLog('Démarrage de l\'initialisation...', 'orange');

forkJoin([
  initDatabase$,
  loadConfig$,
  authenticate$
]).subscribe({
  complete: () => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#e8f5e9';
    completeMessage.style.color = 'green';
    completeMessage.style.fontWeight = 'bold';
    completeMessage.textContent = '✅ Toute l\'initialisation est terminée ! L\'application peut démarrer.';
    addLog('Démarrage de l\'application', 'green');
  },
  error: err => {
    completeMessage.style.display = 'block';
    completeMessage.style.backgroundColor = '#ffebee';
    completeMessage.style.color = 'red';
    completeMessage.textContent = `❌ Erreur d'initialisation: ${err.message}`;
  }
});
```

- Les logs détaillés de chaque processus d'initialisation sont affichés, mais les valeurs sont ignorées.
- Quand tous les processus sont terminés, le message de complétion s'affiche.

## 🎯 Exemple de code pratique 2 : Attente de fin d'upload de fichiers

Un exemple affichant la progression de l'upload de plusieurs fichiers tout en ne notifiant que la complétion.

```ts
import { from, of, concat } from 'rxjs';
import { ignoreElements, tap, delay, mergeMap } from 'rxjs';

// Création de l'UI
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Upload de fichiers';
container.appendChild(title);

const button = document.createElement('button');
button.textContent = 'Démarrer l\'upload';
container.appendChild(button);

const progressArea = document.createElement('div');
progressArea.style.marginTop = '10px';
container.appendChild(progressArea);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.display = 'none';
container.appendChild(result);

interface FileUpload {
  name: string;
  size: number;
}

const files: FileUpload[] = [
  { name: 'document.pdf', size: 2500 },
  { name: 'image.jpg', size: 1800 },
  { name: 'video.mp4', size: 5000 }
];

// Traitement d'upload de fichier (avec affichage de progression)
function uploadFile(file: FileUpload) {
  const fileDiv = document.createElement('div');
  fileDiv.style.marginTop = '5px';
  fileDiv.style.padding = '5px';
  fileDiv.style.border = '1px solid #ccc';
  progressArea.appendChild(fileDiv);

  const progressSteps = [0, 25, 50, 75, 100];

  return from(progressSteps).pipe(
    delay(200),
    tap(progress => {
      fileDiv.textContent = `📄 ${file.name} (${file.size}KB) - ${progress}%`;
      if (progress === 100) {
        fileDiv.style.backgroundColor = '#e8f5e9';
      }
    }),
    ignoreElements() // Ignore les valeurs de progression, ne notifie que la complétion
  );
}

button.addEventListener('click', () => {
  button.disabled = true;
  progressArea.innerHTML = '';
  result.style.display = 'none';

  // Upload séquentiel de tous les fichiers
  from(files).pipe(
    mergeMap(file => uploadFile(file), 2) // Maximum 2 en parallèle
  ).subscribe({
    complete: () => {
      result.style.display = 'block';
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
      result.innerHTML = `
        <strong>✅ Upload terminé</strong><br>
        ${files.length} fichiers ont été uploadés
      `;
      button.disabled = false;
    },
    error: err => {
      result.style.display = 'block';
      result.style.backgroundColor = '#ffebee';
      result.style.color = 'red';
      result.textContent = `❌ Erreur: ${err.message}`;
      button.disabled = false;
    }
  });
});
```

- La progression de chaque fichier est affichée, mais les valeurs de progression elles-mêmes ne sont pas transmises en aval.
- Quand tous les uploads sont terminés, le message de complétion s'affiche.

## 🆚 Comparaison avec des opérateurs similaires

### ignoreElements vs filter(() => false) vs take(0)

```ts
import { of } from 'rxjs';
import { ignoreElements, filter, take } from 'rxjs';

const source$ = of(1, 2, 3);

// ignoreElements: ignore toutes les valeurs, laisse passer la complétion
source$.pipe(
  ignoreElements()
).subscribe({
  next: v => console.log('Valeur:', v),
  complete: () => console.log('ignoreElements: Terminé')
});
// Sortie: ignoreElements: Terminé

// filter(() => false): filtre toutes les valeurs, laisse passer la complétion
source$.pipe(
  filter(() => false)
).subscribe({
  next: v => console.log('Valeur:', v),
  complete: () => console.log('filter: Terminé')
});
// Sortie: filter: Terminé

// take(0): termine immédiatement
source$.pipe(
  take(0)
).subscribe({
  next: v => console.log('Valeur:', v),
  complete: () => console.log('take(0): Terminé')
});
// Sortie: take(0): Terminé
```

| Opérateur | Traitement des valeurs | Notification de complétion | Cas d'utilisation |
|:---|:---|:---|:---|
| `ignoreElements()` | Toutes ignorées | Transmise | **Uniquement complétion nécessaire** (recommandé) |
| `filter(() => false)` | Toutes filtrées | Transmise | Filtrage conditionnel (toutes exclues par hasard) |
| `take(0)` | Termine immédiatement | Transmise | Quand vous voulez terminer immédiatement |

**Recommandation** : Si vous voulez intentionnellement ignorer toutes les valeurs, utilisez `ignoreElements()`. L'intention du code sera plus claire.

## 🔄 Gestion des notifications d'erreur

`ignoreElements` ignore les valeurs mais **transmet les notifications d'erreur**.

```ts
import { throwError, of, concat } from 'rxjs';
import { ignoreElements, delay } from 'rxjs';

const success$ = of(1, 2, 3).pipe(
  delay(100),
  ignoreElements()
);

const error$ = concat(
  of(1, 2, 3),
  throwError(() => new Error('Erreur survenue'))
).pipe(
  ignoreElements()
);

// Cas de succès
success$.subscribe({
  next: v => console.log('Valeur:', v),
  complete: () => console.log('✅ Terminé'),
  error: err => console.error('❌ Erreur:', err.message)
});
// Sortie: ✅ Terminé

// Cas d'erreur
error$.subscribe({
  next: v => console.log('Valeur:', v),
  complete: () => console.log('✅ Terminé'),
  error: err => console.error('❌ Erreur:', err.message)
});
// Sortie: ❌ Erreur: Erreur survenue
```

## ⚠️ Points d'attention

### 1. Les effets de bord sont exécutés

`ignoreElements` ignore les valeurs, mais les effets de bord (comme `tap`) sont exécutés.

```ts
import { of } from 'rxjs';
import { ignoreElements, tap } from 'rxjs';

of(1, 2, 3).pipe(
  tap(v => console.log('Effet de bord:', v)),
  ignoreElements()
).subscribe({
  next: v => console.log('Valeur:', v),
  complete: () => console.log('Terminé')
});
// Sortie:
// Effet de bord: 1
// Effet de bord: 2
// Effet de bord: 3
// Terminé
```

### 2. Utilisation avec des Observables infinis

Avec un Observable infini, la complétion n'arrive jamais, donc la souscription continue indéfiniment.

```ts
import { interval } from 'rxjs';
import { ignoreElements, take } from 'rxjs';

// ❌ Mauvais exemple: ne termine jamais
interval(1000).pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Terminé') // Non appelé
});

// ✅ Bon exemple: terminer avec take
interval(1000).pipe(
  take(5),
  ignoreElements()
).subscribe({
  complete: () => console.log('Terminé') // Appelé après 5 secondes
});
```

### 3. Type TypeScript

Le type de retour de `ignoreElements` est `Observable<never>`.

```ts
import { Observable, of } from 'rxjs';
import { ignoreElements } from 'rxjs';

const numbers$: Observable<number> = of(1, 2, 3);

// Le résultat de ignoreElements est Observable<never>
const result$: Observable<never> = numbers$.pipe(
  ignoreElements()
);

result$.subscribe({
  next: value => {
    // value est de type never, donc ce bloc n'est jamais exécuté
    console.log(value);
  },
  complete: () => console.log('Complétion uniquement')
});
```

### 4. Quand la complétion n'est pas garantie

Si la source ne termine pas, `ignoreElements` ne termine pas non plus.

```ts
import { NEVER } from 'rxjs';
import { ignoreElements } from 'rxjs';

// ❌ NEVER n'émet jamais de complétion ni d'erreur
NEVER.pipe(
  ignoreElements()
).subscribe({
  complete: () => console.log('Terminé') // Non appelé
});
```

## 💡 Patterns de combinaison pratiques

### Pattern 1 : Séquence d'initialisation

```ts
import { of, concat } from 'rxjs';
import { tap, ignoreElements, delay } from 'rxjs';

const initStep1$ = of('Étape 1').pipe(
  tap(console.log),
  delay(1000),
  ignoreElements()
);

const initStep2$ = of('Étape 2').pipe(
  tap(console.log),
  delay(1000),
  ignoreElements()
);

const initStep3$ = of('Étape 3').pipe(
  tap(console.log),
  delay(1000),
  ignoreElements()
);

// Exécuter toutes les étapes séquentiellement
concat(initStep1$, initStep2$, initStep3$).subscribe({
  complete: () => console.log('✅ Toute l\'initialisation terminée')
});
```

### Pattern 2 : Traitement de nettoyage

```ts
import { from, of } from 'rxjs';
import { tap, ignoreElements, mergeMap } from 'rxjs';

interface Resource {
  id: number;
  name: string;
}

const resources: Resource[] = [
  { id: 1, name: 'Base de données' },
  { id: 2, name: 'Cache' },
  { id: 3, name: 'Logger' }
];

from(resources).pipe(
  mergeMap(resource =>
    of(resource).pipe(
      tap(() => console.log(`🧹 Nettoyage de ${resource.name}...`)),
      ignoreElements()
    )
  )
).subscribe({
  complete: () => console.log('✅ Toutes les ressources ont été nettoyées')
});
```

## 📚 Opérateurs associés

- **[filter](./filter)** - Filtrer les valeurs basé sur une condition
- **[take](./take)** - Récupérer uniquement les N premières valeurs
- **[skip](./skip)** - Ignorer les N premières valeurs
- **[tap](../utility/tap)** - Exécuter des effets de bord

## Résumé

L'opérateur `ignoreElements` ignore toutes les valeurs et ne transmet que la complétion et les erreurs.

- ✅ Optimal quand seule la notification de complétion est nécessaire
- ✅ Les effets de bord (tap) sont exécutés
- ✅ Les notifications d'erreur sont également transmises
- ✅ Plus explicite que `filter(() => false)`
- ⚠️ Ne termine jamais avec un Observable infini
- ⚠️ Le type de retour est `Observable<never>`
- ⚠️ Les valeurs sont complètement ignorées, mais les effets de bord sont exécutés

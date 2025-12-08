---
description: "L'opérateur mergeMap transforme chaque valeur en un nouvel Observable et les exécute simultanément, en les fusionnant à plat. Pratique pour l'exécution parallèle de plusieurs requêtes API sans attente, et la gestion de traitements asynchrones imbriqués."
---

# mergeMap - Transformer chaque valeur en Observable et fusionner simultanément

L'opérateur `mergeMap` (aussi appelé `flatMap`) transforme chaque valeur en un nouvel Observable et **les fusionne à plat simultanément**.
C'est très pratique lorsque vous voulez exécuter des requêtes immédiatement sans attendre en séquence, ou pour un traitement asynchrone imbriqué.

## 🔰 Syntaxe de base et utilisation

```ts
import { of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

of('A', 'B', 'C').pipe(
  mergeMap(value =>
    of(`${value} terminé`).pipe(delay(1000))
  )
).subscribe(console.log);

// Sortie (ordre non garanti) :
// A terminé
// B terminé
// C terminé
```

- Un nouvel Observable est généré pour chaque valeur.
- Ces Observables sont **exécutés en parallèle**, et les résultats sont produits sans ordre particulier.

[🌐 Documentation officielle RxJS - mergeMap](https://rxjs.dev/api/operators/mergeMap)

## 💡 Modes d'utilisation typiques

- Envoi de requêtes API à chaque clic de bouton
- Lancer le téléchargement d'un fichier à chaque événement de dépôt de fichier
- Exécution simultanée de tâches asynchrones déclenchées par les actions de l'utilisateur

## 🧠 Exemple de code pratique (avec interface utilisateur)

Exemple où chaque clic sur un bouton déclenche une requête asynchrone (réponse après 2 secondes).

```ts
import { fromEvent, of } from 'rxjs';
import { mergeMap, delay } from 'rxjs';

// Création du bouton
const button = document.createElement('button');
button.textContent = 'Envoyer requête';
document.body.appendChild(button);

// Zone de sortie
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Événement de clic
fromEvent(button, 'click').pipe(
  mergeMap((_, index) => {
    const requestId = index + 1;
    console.log(`Requête ${requestId} démarrée`);
    return of(`Réponse ${requestId}`).pipe(delay(2000));
  })
).subscribe((response) => {
  const div = document.createElement('div');
  div.textContent = `✅ ${response}`;
  output.appendChild(div);
});
```

- Une requête asynchrone est émise immédiatement pour chaque clic.
- **Chaque requête attend 2 secondes individuellement**, les résultats n'arrivent donc pas dans l'ordre.
- Un exemple optimal pour comprendre le traitement parallèle.
